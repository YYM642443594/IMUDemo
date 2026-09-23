#include "serial_parse.h"
#include "rclcpp/rclcpp.hpp"
#include "zhixiang_imu_ros2/msg/imu_data1.hpp"
#include "zhixiang_imu_ros2/msg/imu_data2.hpp"
#include "zhixiang_imu_ros2/msg/imu_data3.hpp"

/* 数据域大小校验, 与协议表格字段偏移保持一致 */
static_assert(sizeof(imu_data1_t) == 87, "imu_data1_t size error");
static_assert(sizeof(imu_data2_t) == 44, "imu_data2_t size error");
static_assert(sizeof(imu_data3_t) == 105, "imu_data3_t size error");

/* serial_port.cpp 中定义 */
extern std::string frame_id;
extern zhixiang_imu_ros2::msg::ImuData1 imu1_msg;
extern zhixiang_imu_ros2::msg::ImuData2 imu2_msg;
extern zhixiang_imu_ros2::msg::ImuData3 imu3_msg;
extern rclcpp::Publisher<zhixiang_imu_ros2::msg::ImuData1>::SharedPtr imu1_pub;
extern rclcpp::Publisher<zhixiang_imu_ros2::msg::ImuData2>::SharedPtr imu2_pub;
extern rclcpp::Publisher<zhixiang_imu_ros2::msg::ImuData3>::SharedPtr imu3_pub;

ParseStruct _parse;
static uint8_t frame_buf[IMU_FRAME_MAX_LEN];

/**
 * CRC16计算: 初值0x3692, 多项式0x8408(反向)
 * 计算范围从帧头开始到数据域的最后一个字节(不含CRC本身)
 */
uint16_t crc16_value_cal(const uint8_t *buf, uint16_t n)
{
    uint16_t crc = 0x3692;

    for (uint16_t i = 0; i < n; i++)
    {
        crc ^= buf[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/**
 * 传感器推送数据1(0x00 0x01)解析并发布
 */
static void publish_data1(const uint8_t *payload)
{
    imu_data1_t d;

    memcpy(&d, payload, sizeof(d));

    imu1_msg.header.stamp = rclcpp::Clock().now();
    imu1_msg.header.frame_id = frame_id;

    imu1_msg.push_freq = d.freq;
    imu1_msg.year = d.year;
    imu1_msg.month = d.month;
    imu1_msg.day = d.day;
    imu1_msg.hour = d.hour;
    imu1_msg.minute = d.minute;
    imu1_msg.second = d.second;
    for (int i = 0; i < 4; i++)
    {
        imu1_msg.acc[i] = d.acc[i];
        imu1_msg.gyr[i] = d.gyr[i];
        imu1_msg.quat[i] = d.quat[i];
    }
    imu1_msg.temperature = d.temp;
    for (int i = 0; i < 3; i++)
    {
        imu1_msg.rpy[i] = d.rpy[i];
        imu1_msg.mag[i] = d.mag[i];
    }

    imu1_pub->publish(imu1_msg);
}

/**
 * 传感器推送数据2(0x00 0x10)解析并发布
 */
static void publish_data2(const uint8_t *payload)
{
    imu_data2_t d;

    memcpy(&d, payload, sizeof(d));

    imu2_msg.header.stamp = rclcpp::Clock().now();
    imu2_msg.header.frame_id = frame_id;

    imu2_msg.timestamp = d.timestamp;
    for (int i = 0; i < 3; i++)
    {
        imu2_msg.acc[i] = d.acc[i];
        imu2_msg.gyr[i] = d.gyr[i];
        imu2_msg.rpy[i] = d.rpy[i];
    }

    imu2_pub->publish(imu2_msg);
}

/**
 * 传感器推送数据3(0x00 0x13)解析并发布
 */
static void publish_data3(const uint8_t *payload)
{
    imu_data3_t d;

    memcpy(&d, payload, sizeof(d));

    imu3_msg.header.stamp = rclcpp::Clock().now();
    imu3_msg.header.frame_id = frame_id;

    imu3_msg.push_freq = d.freq;
    imu3_msg.year = d.year;
    imu3_msg.month = d.month;
    imu3_msg.day = d.day;
    imu3_msg.hour = d.hour;
    imu3_msg.minute = d.minute;
    imu3_msg.second = d.second;
    imu3_msg.timestamp = d.timestamp;
    imu3_msg.pps_lock = d.pps_lock;
    imu3_msg.pps_mode = d.pps_mode;
    for (int i = 0; i < 4; i++)
    {
        imu3_msg.acc[i] = d.acc[i];
        imu3_msg.gyr[i] = d.gyr[i];
        imu3_msg.quat[i] = d.quat[i];
    }
    imu3_msg.temperature = d.temp;
    for (int i = 0; i < 3; i++)
    {
        imu3_msg.rpy[i] = d.rpy[i];
        imu3_msg.mag[i] = d.mag[i];
    }

    imu3_pub->publish(imu3_msg);
}

/**
 * 帧数据提取分发(整帧已存放于frame_buf且通过CRC校验)
 */
static void data_extraction(void)
{
    const uint8_t *payload = frame_buf + IMU_FRAME_HEAD_LEN;

    if (_parse.cmd != IMU_CMD_GENERAL)
        return;

    switch (_parse.id)
    {
    case IMU_MSGID_DATA1:
        if (_parse.payload_len == sizeof(imu_data1_t))
            publish_data1(payload);
        break;

    case IMU_MSGID_DATA2:
        if (_parse.payload_len == sizeof(imu_data2_t))
            publish_data2(payload);
        break;

    case IMU_MSGID_DATA3:
        if (_parse.payload_len == sizeof(imu_data3_t))
            publish_data3(payload);
        break;

    default:
        break;
    }
}

/**
 * 解码状态机: 逐字节输入串口接收数据
 */
void imu_rx(uint8_t data)
{
    static uint16_t frame_idx = 0;

    switch (_parse.state)
    {
    case IMU_PARSE_STATE_WAIT_SYNC1:
        if (data == IMU_FRAME_HEAD1)
        {
            frame_buf[0] = data;
            frame_idx = 1;
            _parse.state = IMU_PARSE_STATE_WAIT_SYNC2;
        }
        break;

    case IMU_PARSE_STATE_WAIT_SYNC2:
        if (data == IMU_FRAME_HEAD2)
        {
            frame_buf[frame_idx++] = data;
            _parse.state = IMU_PARSE_STATE_WAIT_CMD;
        }
        else
        {
            _parse.state = IMU_PARSE_STATE_WAIT_SYNC1;
        }
        break;

    case IMU_PARSE_STATE_WAIT_CMD:
        _parse.cmd = data;
        frame_buf[frame_idx++] = data;
        _parse.state = IMU_PARSE_STATE_WAIT_ID;
        break;

    case IMU_PARSE_STATE_WAIT_ID:
        _parse.id = data;
        frame_buf[frame_idx++] = data;
        _parse.state = IMU_PARSE_STATE_WAIT_LENGTH1;
        break;

    case IMU_PARSE_STATE_WAIT_LENGTH1:
        _parse.length = data; /* 低字节, 小端 */
        frame_buf[frame_idx++] = data;
        _parse.state = IMU_PARSE_STATE_WAIT_LENGTH2;
        break;

    case IMU_PARSE_STATE_WAIT_LENGTH2:
        _parse.length |= (uint16_t)data << 8; /* 高字节 */
        frame_buf[frame_idx++] = data;
        /* 长度为整帧长度: 帧头6 + 数据域 + CRC16(2) */
        if (_parse.length >= (IMU_FRAME_HEAD_LEN + IMU_FRAME_CRC_LEN) &&
            _parse.length <= IMU_FRAME_MAX_LEN)
        {
            _parse.payload_len = _parse.length - IMU_FRAME_HEAD_LEN - IMU_FRAME_CRC_LEN;
            _parse.count = 0;
            _parse.state = IMU_PARSE_STATE_PAYLOAD;
        }
        else
        {
            _parse.state = IMU_PARSE_STATE_WAIT_SYNC1;
        }
        break;

    case IMU_PARSE_STATE_PAYLOAD:
        frame_buf[frame_idx++] = data;
        if (++_parse.count >= (uint16_t)(_parse.payload_len + IMU_FRAME_CRC_LEN))
        {
            uint16_t crc_calc = crc16_value_cal(frame_buf,
                                                (uint16_t)(_parse.length - IMU_FRAME_CRC_LEN));
            uint16_t crc_rx = frame_buf[_parse.length - 2] |
                              ((uint16_t)frame_buf[_parse.length - 1] << 8);

            if (crc_calc == crc_rx)
                data_extraction();

            _parse.state = IMU_PARSE_STATE_WAIT_SYNC1;
        }
        break;

    default:
        _parse.state = IMU_PARSE_STATE_WAIT_SYNC1;
        break;
    }
}
