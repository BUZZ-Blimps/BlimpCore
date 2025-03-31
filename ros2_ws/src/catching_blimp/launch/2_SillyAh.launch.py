from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Define the namespace for this instance
    namespace = 'SillyAh'

    # Path to the main launch file
    catchingblimp_launch_path = os.path.join(
        os.path.dirname(__file__),
        'catchingblimp.launch.py'
    )

    # Include the main launch file with the specified namespace
    include_catchingblimp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(catchingblimp_launch_path),
        launch_arguments={'namespace': namespace}.items(),
    )

    return LaunchDescription([
        include_catchingblimp_launch
    ])
