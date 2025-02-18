from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rl_controller',
            # namespace='turtlesim1',
            executable='controller_node',
            name='sim'
        ),
    ])