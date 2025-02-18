import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu

from .controllers import RobotConfig
from .controllers import RobotController

class ControllerNode(Node):
    def __init__(self):
        """ 
        Initialize the controller node, subscriber, publisher.
        """
        super().__init__('rl_controller')
        
        # Load the robot configuration and initialize the robot controller
        config = RobotConfig.from_yaml('configs/leg_robot.yaml')  
        self.robot_controller = RobotController(config)

        # Create a timer to call the control loop
        self.create_timer(1.0/50.0, self.timer_callback)

        # create a subscriber to the joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.robot_controller.robot.joint_states_callback,
            10
        )

        # create a subscriber to the imu data
        self.imu_sub = self.create_subscription(
            Imu,
            'imu',
            self.robot_controller.robot.imu_callback,
            10
        )

        # create a publisher to the joint commands
        self.joint_cmd_pub = self.create_publisher(JointState, 'joint_commands', 10)

    def timer_callback(self):
        """
        Timer callback to compute the action.
        """
        # Compute the action from the robot controller
        cmd = np.ndarray([1.5, 0.0, 0.0], shape=(3,))
        joint_cmds = self.robot_controller.compute(cmd)

        # Compose and publish the joint command message
        msg = JointState()
        msg.position = joint_cmds
        msg.velocity = [0.0] * len(joint_cmds)
        msg.effort = [0.0] * len(joint_cmds)
        self.joint_cmd_pub.publish(msg)


        


if __name__ == '__main__':
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()