from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare the 'namespace' launch argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='BurnCream',
        description='Namespace for the nodes'
    )

    # Define the catching_blimp_node with the namespace from the launch argument
    catching_blimp_node = Node(
        package='catching_blimp',
        executable='catching_blimp_node',
        name='catching_blimp_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            os.path.join(
                get_package_share_directory('catching_blimp'),
                'param',
                'pid_config.yaml'
            )
        ],
        output='screen'
    )

    battery_monitor_node = Node(
        package='catching_blimp',
        executable='battery_monitor_node.py',
        name='battery_monitor_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        catching_blimp_node,
        battery_monitor_node
    ])
