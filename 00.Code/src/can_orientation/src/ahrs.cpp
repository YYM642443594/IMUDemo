#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <cstring>
#include <string>

#include "can_orientation/msg/can_imu_data.hpp"

/* ===== CAN标准帧ID定义 (用户协议 3.3.2 CAN输出标准帧格式) ===== */
#define IMU_CAN_ID_TIMESTAMP    0x091 /* Timestamp uint64 (us)           */
#define IMU_CAN_ID_ROLL_PITCH   0x092 /* Roll, Pitch (float, deg)        */
#define IMU_CAN_ID_YAW_TEMP     0x093 /* Yaw (deg), Temp (degC)          */
#define IMU_CAN_ID_Q12          0x094 /* q1(w), q2(x) (float)            */
#define IMU_CAN_ID_Q34          0x095 /* q3(y), q4(z) (float)            */
#define IMU_CAN_ID_ACC_XY       0x096 /* X_Acc, Y_Acc (float, m/s^2)     */
#define IMU_CAN_ID_ACC_Z_GYRO_X 0x097 /* Z_Acc (m/s^2), X_Gyro (deg/s)   */
#define IMU_CAN_ID_GYRO_YZ      0x098 /* Y_Gyro, Z_Gyro (float, deg/s)   */

/* 0x091~0x098 每帧有效数据均为8字节 */
#define CAN_DATA_LEN 8
#define DEG_TO_RAD (M_PI / 180.0)

class CanOrientationNode : public rclcpp::Node
{
public:
    CanOrientationNode() : Node("can_orientation_node")
    {
        this->declare_parameter<std::string>("can_interface", "can0");
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<std::string>("imu_topic", "imu/data");
        this->declare_parameter<std::string>("can_data_topic", "/can_imu_data");

        std::string can_interface, frame_id, imu_topic, can_data_topic;
        this->get_parameter("can_interface", can_interface);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("imu_topic", imu_topic);
        this->get_parameter("can_data_topic", can_data_topic);

        /* 发布器: sensor_msgs/Imu(标准接口,兼容RViz) + CanImuData(0x91~0x98全字段) */
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
        data_pub_ = this->create_publisher<can_orientation::msg::CanImuData>(can_data_topic, 10);

        imu_msg_.header.frame_id = frame_id;
        imu_msg_.orientation_covariance[0] = 1e-3;
        imu_msg_.orientation_covariance[4] = 1e-3;
        imu_msg_.orientation_covariance[8] = 1e-3;
        can_msg_.header.frame_id = frame_id;

        /* ===== SocketCAN初始化 ===== */
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "CAN socket create failed");
            rclcpp::shutdown();
            return;
        }

        struct ifreq ifr {};
        std::strncpy(ifr.ifr_name, can_interface.c_str(), IFNAMSIZ - 1);
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_FATAL(this->get_logger(),
                         "CAN interface '%s' not found (ip link show?)", can_interface.c_str());
            rclcpp::shutdown();
            return;
        }

        struct sockaddr_can addr {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "CAN socket bind failed");
            rclcpp::shutdown();
            return;
        }

        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        RCLCPP_INFO(this->get_logger(),
                    "ZhiXiang IMU CAN driver started: if=%s, pub=[%s, %s], frame_id=%s",
                    can_interface.c_str(), imu_topic.c_str(),
                    can_data_topic.c_str(), frame_id.c_str());

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&CanOrientationNode::readCanData, this));
    }

    ~CanOrientationNode()
    {
        if (socket_fd_ >= 0)
            close(socket_fd_);
    }

private:
    /* 收齐 0x091~0x098 全部8帧后发布一次, 位掩码容忍乱序/丢帧 */
    void readCanData()
    {
        struct can_frame frame;

        while (read(socket_fd_, &frame, sizeof(frame)) > 0)
        {
            if (frame.can_dlc != CAN_DATA_LEN)
                continue;

            switch (frame.can_id)
            {
            case IMU_CAN_ID_TIMESTAMP:
                std::memcpy(&can_msg_.timestamp_us, frame.data, 8);
                collected_ |= (1 << 0);
                break;

            case IMU_CAN_ID_ROLL_PITCH:
            {
                float roll, pitch;
                std::memcpy(&roll, frame.data, 4);
                std::memcpy(&pitch, frame.data + 4, 4);
                can_msg_.roll = roll;
                can_msg_.pitch = pitch;
                collected_ |= (1 << 1);
                break;
            }

            case IMU_CAN_ID_YAW_TEMP:
            {
                float yaw, temp;
                std::memcpy(&yaw, frame.data, 4);
                std::memcpy(&temp, frame.data + 4, 4);
                can_msg_.yaw = yaw;
                can_msg_.temperature = temp;
                collected_ |= (1 << 2);
                break;
            }

            case IMU_CAN_ID_Q12:
            {
                float q1, q2;
                std::memcpy(&q1, frame.data, 4);
                std::memcpy(&q2, frame.data + 4, 4);
                can_msg_.quaternion[0] = q1; /* w */
                can_msg_.quaternion[1] = q2; /* x */
                collected_ |= (1 << 3);
                break;
            }

            case IMU_CAN_ID_Q34:
            {
                float q3, q4;
                std::memcpy(&q3, frame.data, 4);
                std::memcpy(&q4, frame.data + 4, 4);
                can_msg_.quaternion[2] = q3; /* y */
                can_msg_.quaternion[3] = q4; /* z */
                collected_ |= (1 << 4);
                break;
            }

            case IMU_CAN_ID_ACC_XY:
            {
                float ax, ay;
                std::memcpy(&ax, frame.data, 4);
                std::memcpy(&ay, frame.data + 4, 4);
                can_msg_.acc[0] = ax;
                can_msg_.acc[1] = ay;
                collected_ |= (1 << 5);
                break;
            }

            case IMU_CAN_ID_ACC_Z_GYRO_X:
            {
                float az, gx;
                std::memcpy(&az, frame.data, 4);
                std::memcpy(&gx, frame.data + 4, 4);
                can_msg_.acc[2] = az;
                can_msg_.gyro[0] = gx;
                collected_ |= (1 << 6);
                break;
            }

            case IMU_CAN_ID_GYRO_YZ:
            {
                float gy, gz;
                std::memcpy(&gy, frame.data, 4);
                std::memcpy(&gz, frame.data + 4, 4);
                can_msg_.gyro[1] = gy;
                can_msg_.gyro[2] = gz;
                collected_ |= (1 << 7);
                break;
            }

            default:
                break; /* 非协议帧忽略 */
            }

            if (collected_ == COLLECT_ALL)
            {
                fillAndPublish();
                collected_ = 0;
            }
        }
    }

    void fillAndPublish()
    {
        /* sensor_msgs/Imu: 协议直接输出四元数(q1=w), 角速度deg/s转rad/s, 加速度m/s^2原样 */
        imu_msg_.header.stamp = this->now();
        imu_msg_.orientation.w = can_msg_.quaternion[0];
        imu_msg_.orientation.x = can_msg_.quaternion[1];
        imu_msg_.orientation.y = can_msg_.quaternion[2];
        imu_msg_.orientation.z = can_msg_.quaternion[3];
        imu_msg_.angular_velocity.x = can_msg_.gyro[0] * DEG_TO_RAD;
        imu_msg_.angular_velocity.y = can_msg_.gyro[1] * DEG_TO_RAD;
        imu_msg_.angular_velocity.z = can_msg_.gyro[2] * DEG_TO_RAD;
        imu_msg_.linear_acceleration.x = can_msg_.acc[0];
        imu_msg_.linear_acceleration.y = can_msg_.acc[1];
        imu_msg_.linear_acceleration.z = can_msg_.acc[2];
        imu_pub_->publish(imu_msg_);

        /* 全字段消息: 保留协议原始单位 (deg, degC, deg/s) */
        can_msg_.header.stamp = this->now();
        data_pub_->publish(can_msg_);
    }

    /* ===== 成员变量 ===== */
    int socket_fd_{-1};
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<can_orientation::msg::CanImuData>::SharedPtr data_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Imu imu_msg_;
    can_orientation::msg::CanImuData can_msg_;

    uint8_t collected_{0};
    static constexpr uint8_t COLLECT_ALL = 0xFF;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanOrientationNode>());
    rclcpp::shutdown();
    return 0;
}
