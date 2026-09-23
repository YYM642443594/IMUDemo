#include <iostream>
#include <string>
#include <thread>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <poll.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

#include "rclcpp/rclcpp.hpp"
#include "zhixiang_imu_ros2/msg/imu_data1.hpp"
#include "zhixiang_imu_ros2/msg/imu_data2.hpp"
#include "zhixiang_imu_ros2/msg/imu_data3.hpp"
#include "zhixiang_imu_ros2/srv/set_push.hpp"
#include "serial_parse.h"

#define BUF_SIZE (1024)

std::string serial_port;
int baud_rate;
std::string frame_id;
std::string imu_topic1;
std::string imu_topic2;
std::string imu_topic3;

using SetPush = zhixiang_imu_ros2::srv::SetPush;

zhixiang_imu_ros2::msg::ImuData1 imu1_msg;
zhixiang_imu_ros2::msg::ImuData2 imu2_msg;
zhixiang_imu_ros2::msg::ImuData3 imu3_msg;
rclcpp::Publisher<zhixiang_imu_ros2::msg::ImuData1>::SharedPtr imu1_pub;
rclcpp::Publisher<zhixiang_imu_ros2::msg::ImuData2>::SharedPtr imu2_pub;
rclcpp::Publisher<zhixiang_imu_ros2::msg::ImuData3>::SharedPtr imu3_pub;

class IMUPublisher : public rclcpp::Node
{
public:
    IMUPublisher() : Node("IMU_publisher")
    {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 921600);
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<std::string>("imu_topic1", "/imu_data1");
        this->declare_parameter<std::string>("imu_topic2", "/imu_data2");
        this->declare_parameter<std::string>("imu_topic3", "/imu_data3");

        this->get_parameter("serial_port", serial_port);
        this->get_parameter("baud_rate", baud_rate);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("imu_topic1", imu_topic1);
        this->get_parameter("imu_topic2", imu_topic2);
        this->get_parameter("imu_topic3", imu_topic3);

        RCLCPP_INFO(this->get_logger(), "serial_port: %s", serial_port.c_str());
        RCLCPP_INFO(this->get_logger(), "baud_rate: %d", baud_rate);
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "imu_topic1: %s", imu_topic1.c_str());
        RCLCPP_INFO(this->get_logger(), "imu_topic2: %s", imu_topic2.c_str());
        RCLCPP_INFO(this->get_logger(), "imu_topic3: %s", imu_topic3.c_str());

        imu1_pub = this->create_publisher<zhixiang_imu_ros2::msg::ImuData1>(imu_topic1, 10);
        imu2_pub = this->create_publisher<zhixiang_imu_ros2::msg::ImuData2>(imu_topic2, 10);
        imu3_pub = this->create_publisher<zhixiang_imu_ros2::msg::ImuData3>(imu_topic3, 10);

        fd = open_serial(serial_port, baud_rate);
        if (fd < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port, node exit");
            rclcpp::shutdown();
            return;
        }

        configure_low_latency(fd);

        /* 推送开关控制服务: 开启/关闭传感器数据1/2/3等报文推送(0x00 0x11指令) */
        set_push_srv = this->create_service<SetPush>(
            "imu_set_push",
            [this](const std::shared_ptr<SetPush::Request> req,
                   std::shared_ptr<SetPush::Response> res)
            {
                res->success = send_push_ctrl(req->data_id, req->enable, req->save_to_flash);
            });
        RCLCPP_INFO(this->get_logger(),
                    "Service [imu_set_push] ready, usage: "
                    "ros2 service call /imu_set_push zhixiang_imu_ros2/srv/SetPush "
                    "\"{data_id: 1, enable: true, save_to_flash: false}\" "
                    "(data_id: 0x01=DATA1 0x10=DATA2 0x13=DATA3 0x02=DEV_STATUS)");

        /* 串口读取线程 */
        read_thread = std::thread([this]()
                                  {
            while (rclcpp::ok())
                imu_read(); });
    }

    ~IMUPublisher()
    {
        if (read_thread.joinable())
            read_thread.join();
    }

private:
    int fd = 0;
    uint8_t buf[BUF_SIZE] = {0};
    std::thread read_thread;
    rclcpp::Service<SetPush>::SharedPtr set_push_srv;

    /**
     * 发送推送报文ID控制指令(0x00 0x11), 见协议3.2.3.7
     * 帧格式: AA 55 00 11 Op 0C 00 CMD ID 开关 CRC16(小端)
     * 仅支持通用CMD(0x00)下的报文ID: 0x01/0x02/0x10/0x13
     */
    bool send_push_ctrl(uint8_t data_id, bool enable, bool flash)
    {
        if (fd < 0)
            return false;

        if (data_id != IMU_MSGID_DATA1 && data_id != IMU_MSGID_DEV_STATUS &&
            data_id != IMU_MSGID_DATA2 && data_id != IMU_MSGID_DATA3)
        {
            RCLCPP_WARN(this->get_logger(), "Unsupported push msg id: 0x%02X", data_id);
            return false;
        }

        uint8_t frame[12];
        frame[0] = IMU_FRAME_HEAD1;
        frame[1] = IMU_FRAME_HEAD2;
        frame[2] = IMU_CMD_GENERAL;
        frame[3] = IMU_MSGID_PUSH_CTRL;
        frame[4] = flash ? IMU_OP_FLASH : IMU_OP_RAM;
        frame[5] = 0x0C; /* 整帧长度12字节, 小端 */
        frame[6] = 0x00;
        frame[7] = IMU_CMD_GENERAL; /* 目标报文CMD */
        frame[8] = data_id;         /* 目标报文ID */
        frame[9] = enable ? 0x01 : 0x00;
        uint16_t crc = crc16_value_cal(frame, 10);
        frame[10] = crc & 0xFF;
        frame[11] = crc >> 8;

        int n = write(fd, frame, sizeof(frame));
        RCLCPP_INFO(this->get_logger(),
                    "Send push ctrl: msg=0x%02X %s(%s) [%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X]",
                    data_id, enable ? "ON" : "OFF", flash ? "FLASH" : "RAM",
                    frame[0], frame[1], frame[2], frame[3], frame[4],
                    frame[5], frame[6], frame[7], frame[8], frame[9], frame[10], frame[11]);

        return n == (int)sizeof(frame);
    }

    void imu_read(void)
    {
        struct pollfd p;
        p.fd = fd;
        p.events = POLLIN;

        int rpoll = poll(&p, 1, 5);
        if (rpoll <= 0)
            return;

        int n = read(fd, buf, sizeof(buf));
        for (int i = 0; i < n; i++)
        {
            imu_rx(buf[i]);
        }
    }

    /* 串口低延迟模式配置(可选优化, 失败仅告警不影响使用) */
    void configure_low_latency(int serial_fd)
    {
        struct serial_struct ser_info;

        if (ioctl(serial_fd, TIOCGSERIAL, &ser_info) < 0)
        {
            RCLCPP_WARN(this->get_logger(), "Get serial info failed: %s", strerror(errno));
            return;
        }

        ser_info.flags |= ASYNC_LOW_LATENCY;

        if (ioctl(serial_fd, TIOCSSERIAL, &ser_info) < 0)
        {
            RCLCPP_WARN(this->get_logger(), "Set low latency mode failed: %s", strerror(errno));
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Low latency mode enabled");
    }

    int open_serial(const std::string &port, int baud)
    {
        const char *port_device = port.c_str();
        int serial_fd = open(port_device, O_RDWR | O_NOCTTY | O_SYNC);

        if (serial_fd == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port: %s", strerror(errno));
            return -1;
        }

        if (fcntl(serial_fd, F_SETFL, O_NONBLOCK) < 0)
            RCLCPP_ERROR(this->get_logger(), "fcntl failed: %s", strerror(errno));

        struct termios options;
        memset(&options, 0, sizeof(options));
        tcgetattr(serial_fd, &options);

        switch (baud)
        {
        case 9600:
            cfsetispeed(&options, B9600);
            cfsetospeed(&options, B9600);
            break;
        case 19200:
            cfsetispeed(&options, B19200);
            cfsetospeed(&options, B19200);
            break;
        case 38400:
            cfsetispeed(&options, B38400);
            cfsetospeed(&options, B38400);
            break;
        case 57600:
            cfsetispeed(&options, B57600);
            cfsetospeed(&options, B57600);
            break;
        case 115200:
            cfsetispeed(&options, B115200);
            cfsetospeed(&options, B115200);
            break;
        case 460800:
            cfsetispeed(&options, B460800);
            cfsetospeed(&options, B460800);
            break;
        case 921600:
            cfsetispeed(&options, B921600);
            cfsetospeed(&options, B921600);
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unsupported baud rate: %d", baud);
            close(serial_fd);
            return -1;
        }

        options.c_cflag &= ~PARENB; /* 无校验 */
        options.c_cflag &= ~CSTOPB; /* 1位停止位 */
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= HUPCL;
        options.c_cflag |= CS8; /* 8位数据位 */
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag |= CREAD | CLOCAL;

        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_iflag &= ~(INLCR | ICRNL);

        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        options.c_oflag &= ~OPOST;
        options.c_oflag &= ~(ONLCR | OCRNL);

        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;
        tcsetattr(serial_fd, TCSANOW, &options);

        return serial_fd;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPublisher>());
    rclcpp::shutdown();

    return 0;
}
