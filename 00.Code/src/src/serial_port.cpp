#include <iostream>
#include <string>
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
#include "serial_parse.h"

#define BUF_SIZE (1024)

std::string serial_port;
int baud_rate;
std::string frame_id;
std::string imu_topic1;
std::string imu_topic2;
std::string imu_topic3;

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

        while (rclcpp::ok())
            imu_read();
    }

private:
    int fd = 0;
    uint8_t buf[BUF_SIZE] = {0};

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

        options.c_cflag &= ~PARENB;  /* 无校验 */
        options.c_cflag &= ~CSTOPB;  /* 1位停止位 */
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= HUPCL;
        options.c_cflag |= CS8;      /* 8位数据位 */
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
