from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the other launch files
    remote_controller_pkg_share = get_package_share_directory('remote_controller')
    rl_controller_pkg_share = get_package_share_directory('rl_controller')
    remote_control_launch = os.path.join(remote_controller_pkg_share, 'launch', 'remote_controller.launch.py')
    rl_controller_launch = os.path.join(rl_controller_pkg_share, 'launch', 'robot_controller.launch.py')

    return LaunchDescription([
        # Include the control system launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(remote_control_launch)
        ),
        # Include the sensor system launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rl_controller_launch)
        ),
        # Optionally, add more nodes or actions here
    ])