#include <memory>
#include <iostream>
#include <iomanip>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "zhixiang_imu_ros2/msg/imu_data1.hpp"
#include "zhixiang_imu_ros2/msg/imu_data2.hpp"
#include "zhixiang_imu_ros2/msg/imu_data3.hpp"

using namespace std;

/**
 * 传感器推送数据1(0x00 0x01)话题回调
 */
void topic_callback1(const zhixiang_imu_ros2::msg::ImuData1::SharedPtr msg)
{
    cout << "[ImuData1]" << "\n";
    cout << "\theader:" << "\n";
    cout << "\t\tstamp:" << "\n";
    cout << "\t\t  secs:" << msg->header.stamp.sec << "\n";
    cout << "\t\t  nanosecs:" << msg->header.stamp.nanosec << "\n";
    cout << "\t\tframe_id:" << msg->header.frame_id << "\n";
    cout << "\tpush_freq: " << msg->push_freq << "Hz" << "\n";
    cout << "\ttime: " << msg->year << "-" << (int)msg->month << "-" << (int)msg->day
         << " " << (int)msg->hour << ":" << (int)msg->minute << ":" << (int)msg->second << "\n";
    cout << "\tacc_x: " << fixed << setprecision(6) << msg->acc[0] << "\n";
    cout << "\tacc_y: " << fixed << setprecision(6) << msg->acc[1] << "\n";
    cout << "\tacc_z: " << fixed << setprecision(6) << msg->acc[2] << "\n";
    cout << "\tacc_a: " << fixed << setprecision(6) << msg->acc[3] << "\n";
    cout << "\tgyro_x: " << fixed << setprecision(6) << msg->gyr[0] << "\n";
    cout << "\tgyro_y: " << fixed << setprecision(6) << msg->gyr[1] << "\n";
    cout << "\tgyro_z: " << fixed << setprecision(6) << msg->gyr[2] << "\n";
    cout << "\tgyro_w: " << fixed << setprecision(6) << msg->gyr[3] << "\n";
    cout << "\ttemp: " << fixed << setprecision(4) << msg->temperature << "\n";
    cout << "\troll: " << fixed << setprecision(6) << msg->rpy[0] << "\n";
    cout << "\tpitch: " << fixed << setprecision(6) << msg->rpy[1] << "\n";
    cout << "\tyaw: " << fixed << setprecision(6) << msg->rpy[2] << "\n";
    cout << "\tq1: " << fixed << setprecision(6) << msg->quat[0] << "\n";
    cout << "\tq2: " << fixed << setprecision(6) << msg->quat[1] << "\n";
    cout << "\tq3: " << fixed << setprecision(6) << msg->quat[2] << "\n";
    cout << "\tq4: " << fixed << setprecision(6) << msg->quat[3] << "\n";
    cout << "\tmag_x: " << fixed << setprecision(6) << msg->mag[0] << "\n";
    cout << "\tmag_y: " << fixed << setprecision(6) << msg->mag[1] << "\n";
    cout << "\tmag_z: " << fixed << setprecision(6) << msg->mag[2] << "\n";
    cout << "---" << endl;
}

/**
 * 传感器推送数据2(0x00 0x10)话题回调
 */
void topic_callback2(const zhixiang_imu_ros2::msg::ImuData2::SharedPtr msg)
{
    cout << "[ImuData2]" << "\n";
    cout << "\theader:" << "\n";
    cout << "\t\tstamp:" << "\n";
    cout << "\t\t  secs:" << msg->header.stamp.sec << "\n";
    cout << "\t\t  nanosecs:" << msg->header.stamp.nanosec << "\n";
    cout << "\t\tframe_id:" << msg->header.frame_id << "\n";
    cout << "\ttimestamp: " << msg->timestamp << "us" << "\n";
    cout << "\tacc_x: " << fixed << setprecision(6) << msg->acc[0] << "\n";
    cout << "\tacc_y: " << fixed << setprecision(6) << msg->acc[1] << "\n";
    cout << "\tacc_z: " << fixed << setprecision(6) << msg->acc[2] << "\n";
    cout << "\tgyro_x: " << fixed << setprecision(6) << msg->gyr[0] << "\n";
    cout << "\tgyro_y: " << fixed << setprecision(6) << msg->gyr[1] << "\n";
    cout << "\tgyro_z: " << fixed << setprecision(6) << msg->gyr[2] << "\n";
    cout << "\troll: " << fixed << setprecision(6) << msg->rpy[0] << "\n";
    cout << "\tpitch: " << fixed << setprecision(6) << msg->rpy[1] << "\n";
    cout << "\tyaw: " << fixed << setprecision(6) << msg->rpy[2] << "\n";
    cout << "---" << endl;
}

/**
 * 传感器推送数据3(0x00 0x13)话题回调
 */
void topic_callback3(const zhixiang_imu_ros2::msg::ImuData3::SharedPtr msg)
{
    cout << "[ImuData3]" << "\n";
    cout << "\theader:" << "\n";
    cout << "\t\tstamp:" << "\n";
    cout << "\t\t  secs:" << msg->header.stamp.sec << "\n";
    cout << "\t\t  nanosecs:" << msg->header.stamp.nanosec << "\n";
    cout << "\t\tframe_id:" << msg->header.frame_id << "\n";
    cout << "\tpush_freq: " << msg->push_freq << "Hz" << "\n";
    cout << "\ttime: " << msg->year << "-" << (int)msg->month << "-" << (int)msg->day
         << " " << (int)msg->hour << ":" << (int)msg->minute << ":" << (int)msg->second << "\n";
    cout << "\ttimestamp: " << msg->timestamp << "us" << "\n";
    cout << "\tpps_lock: " << (int)msg->pps_lock << "\n";
    cout << "\tpps_mode: " << (int)msg->pps_mode << "\n";
    cout << "\tacc_x: " << fixed << setprecision(6) << msg->acc[0] << "\n";
    cout << "\tacc_y: " << fixed << setprecision(6) << msg->acc[1] << "\n";
    cout << "\tacc_z: " << fixed << setprecision(6) << msg->acc[2] << "\n";
    cout << "\tacc_a: " << fixed << setprecision(6) << msg->acc[3] << "\n";
    cout << "\tgyro_x: " << fixed << setprecision(6) << msg->gyr[0] << "\n";
    cout << "\tgyro_y: " << fixed << setprecision(6) << msg->gyr[1] << "\n";
    cout << "\tgyro_z: " << fixed << setprecision(6) << msg->gyr[2] << "\n";
    cout << "\tgyro_w: " << fixed << setprecision(6) << msg->gyr[3] << "\n";
    cout << "\ttemp: " << fixed << setprecision(4) << msg->temperature << "\n";
    cout << "\troll: " << fixed << setprecision(6) << msg->rpy[0] << "\n";
    cout << "\tpitch: " << fixed << setprecision(6) << msg->rpy[1] << "\n";
    cout << "\tyaw: " << fixed << setprecision(6) << msg->rpy[2] << "\n";
    cout << "\tq1: " << fixed << setprecision(6) << msg->quat[0] << "\n";
    cout << "\tq2: " << fixed << setprecision(6) << msg->quat[1] << "\n";
    cout << "\tq3: " << fixed << setprecision(6) << msg->quat[2] << "\n";
    cout << "\tq4: " << fixed << setprecision(6) << msg->quat[3] << "\n";
    cout << "\tmag_x: " << fixed << setprecision(6) << msg->mag[0] << "\n";
    cout << "\tmag_y: " << fixed << setprecision(6) << msg->mag[1] << "\n";
    cout << "\tmag_z: " << fixed << setprecision(6) << msg->mag[2] << "\n";
    cout << "---" << endl;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("imu_listener");

    nh->declare_parameter<std::string>("imu_topic1", "/imu_data1");
    nh->declare_parameter<std::string>("imu_topic2", "/imu_data2");
    nh->declare_parameter<std::string>("imu_topic3", "/imu_data3");

    std::string topic1, topic2, topic3;
    nh->get_parameter("imu_topic1", topic1);
    nh->get_parameter("imu_topic2", topic2);
    nh->get_parameter("imu_topic3", topic3);

    auto sub1 = nh->create_subscription<zhixiang_imu_ros2::msg::ImuData1>(
        topic1, 10, topic_callback1);
    auto sub2 = nh->create_subscription<zhixiang_imu_ros2::msg::ImuData2>(
        topic2, 10, topic_callback2);
    auto sub3 = nh->create_subscription<zhixiang_imu_ros2::msg::ImuData3>(
        topic3, 10, topic_callback3);

    rclcpp::spin(nh);
    rclcpp::shutdown();

    return 0;
}
