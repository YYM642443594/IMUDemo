import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('can_orientation'), 'config', 'can_imu.yaml')

    return LaunchDescription([
        Node(
            package='can_orientation',
            executable='can_orientation_node',
            name='can_orientation_node',
            output='screen',
            parameters=[config],
        ),
    ])
