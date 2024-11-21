import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    blimp_name = 'SantasWeggie'

    return LaunchDescription([
        Node(
            package='imu_test',
            executable='imu_test_node',
            name='imu_test_node',
            namespace=blimp_name,
            parameters=[
                os.path.join(get_package_share_directory('imu_test'), 'calibration', '{}.yaml'.format(blimp_name))
            ],
            output='screen',
            emulate_tty=True
        )
    ])
