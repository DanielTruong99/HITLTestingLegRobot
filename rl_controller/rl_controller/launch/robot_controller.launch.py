import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share_path = get_package_share_directory("rl_controller")
    default_config_file_path = os.path.join(pkg_share_path, "configs", "leg_robot.yaml")

    return LaunchDescription(
        [
            Node(
                package="rl_controller",
                executable="controller_node",
                name="sim",
                parameters=[{"config_file": default_config_file_path}],
            ),
        ]
    )
