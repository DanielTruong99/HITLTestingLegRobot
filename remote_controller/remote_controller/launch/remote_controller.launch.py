# import os
from launch import LaunchDescription
from launch_ros.actions import Node

# from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    remote_controller = Node(
        package="rl_controller",
        executable="remote_controller_node",
        name="remote_controller",
    )
    

    joystick_driver = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {"dev": "/dev/input/js0"},
            {"deadzone": 0.1},
            {"autorepeat_rate": 5},
        ],
    )


    return LaunchDescription([joystick_driver, remote_controller])
