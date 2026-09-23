##launch file
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('zhixiang_imu_ros2'),
        'config',
        'imu_config.yaml')

    return LaunchDescription([
        Node(
            package='zhixiang_imu_ros2',
            executable='talker',
            name='IMU_publisher',
            parameters=[config],
            output='screen',
        ),
        Node(
            package='zhixiang_imu_ros2',
            executable='listener',
            name='imu_listener',
            parameters=[config],
            output='screen',
        ),
    ])
