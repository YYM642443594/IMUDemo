#include <memory>
#include <iostream>
#include <iomanip>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "can_orientation/msg/can_imu_data.hpp"

using namespace std;

/**
 * CAN全字段数据(0x091~0x098)话题回调
 */
void topic_callback_data(const can_orientation::msg::CanImuData::SharedPtr msg)
{
    cout << "[CanImuData]" << "\n";
    cout << "\theader:" << "\n";
    cout << "\tstamp:" << "\n";
    cout << "\tsecs:" << msg->header.stamp.sec << "\n";
    cout << "\tnanosecs:" << msg->header.stamp.nanosec << "\n";
    cout << "\tframe_id:" << msg->header.frame_id << "\n";
    cout << "\ttimestamp: " << msg->timestamp_us << "us" << "\n";
    cout << "\tacc_x: " << fixed << setprecision(6) << msg->acc[0] << "\n";
    cout << "\tacc_y: " << fixed << setprecision(6) << msg->acc[1] << "\n";
    cout << "\tacc_z: " << fixed << setprecision(6) << msg->acc[2] << "\n";
    cout << "\tgyro_x: " << fixed << setprecision(6) << msg->gyro[0] << "\n";
    cout << "\tgyro_y: " << fixed << setprecision(6) << msg->gyro[1] << "\n";
    cout << "\tgyro_z: " << fixed << setprecision(6) << msg->gyro[2] << "\n";
    cout << "\ttemp: " << fixed << setprecision(4) << msg->temperature << "\n";
    cout << "\troll: " << fixed << setprecision(6) << msg->roll << "\n";
    cout << "\tpitch: " << fixed << setprecision(6) << msg->pitch << "\n";
    cout << "\tyaw: " << fixed << setprecision(6) << msg->yaw << "\n";
    cout << "\tq1(w): " << fixed << setprecision(6) << msg->quaternion[0] << "\n";
    cout << "\tq2(x): " << fixed << setprecision(6) << msg->quaternion[1] << "\n";
    cout << "\tq3(y): " << fixed << setprecision(6) << msg->quaternion[2] << "\n";
    cout << "\tq4(z): " << fixed << setprecision(6) << msg->quaternion[3] << "\n";
    cout << "---" << endl;
}

/**
 * 标准Imu话题回调 (rad/s, m/s^2)
 */
void topic_callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    cout << "[Imu]" << "\n";
    cout << "\theader:" << "\n";
    cout << "\tstamp:" << "\n";
    cout << "\tsecs:" << msg->header.stamp.sec << "\n";
    cout << "\tnanosecs:" << msg->header.stamp.nanosec << "\n";
    cout << "\tframe_id:" << msg->header.frame_id << "\n";
    cout << "\torientation w: " << fixed << setprecision(6) << msg->orientation.w << "\n";
    cout << "\torientation x: " << fixed << setprecision(6) << msg->orientation.x << "\n";
    cout << "\torientation y: " << fixed << setprecision(6) << msg->orientation.y << "\n";
    cout << "\torientation z: " << fixed << setprecision(6) << msg->orientation.z << "\n";
    cout << "\tangular_velocity_x: " << fixed << setprecision(6) << msg->angular_velocity.x << "\n";
    cout << "\tangular_velocity_y: " << fixed << setprecision(6) << msg->angular_velocity.y << "\n";
    cout << "\tangular_velocity_z: " << fixed << setprecision(6) << msg->angular_velocity.z << "\n";
    cout << "\tlinear_acceleration_x: " << fixed << setprecision(6) << msg->linear_acceleration.x << "\n";
    cout << "\tlinear_acceleration_y: " << fixed << setprecision(6) << msg->linear_acceleration.y << "\n";
    cout << "\tlinear_acceleration_z: " << fixed << setprecision(6) << msg->linear_acceleration.z << "\n";
    cout << "---" << endl;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("can_imu_listener");

    nh->declare_parameter<std::string>("can_data_topic", "/can_imu_data");
    nh->declare_parameter<std::string>("imu_topic", "imu/data");

    std::string can_data_topic, imu_topic;
    nh->get_parameter("can_data_topic", can_data_topic);
    nh->get_parameter("imu_topic", imu_topic);

    auto sub2 = nh->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 10, topic_callback_imu);
    auto sub1 = nh->create_subscription<can_orientation::msg::CanImuData>(
        can_data_topic, 10, topic_callback_data);

    rclcpp::spin(nh);
    rclcpp::shutdown();

    return 0;
}
