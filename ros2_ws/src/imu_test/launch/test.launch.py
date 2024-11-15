import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_test',
            executable='imu_test_node',
            name='imu_test_node',
            namespace='GameChamber',
            output='screen'
        )
    ])
