import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    

    catching_blimp_cmd = Node(
        package='ros2_blimp',
        executable='ros2_blimp_node',
        name='ros2_blimp_node',
        namespace='BurnCream',
        parameters=[
            os.path.join(get_package_share_directory('ros2_blimp'), 'param', 'pid_config.yaml')
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(catching_blimp_cmd)
    
    return ld