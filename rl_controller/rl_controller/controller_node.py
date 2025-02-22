import rclpy
import numpy as np
import sys
import signal
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu

from .controllers import RobotConfig
from .controllers import RobotController, RobotEvent


class ControllerNode(Node):
    def __init__(self):
        """
        Initialize the controller node, subscriber, publisher.
        """
        super().__init__("rl_controller")

        # Load the robot configuration and initialize the robot controller
        self.declare_parameter("config_file", "configs/leg_robot.yaml")
        config_file = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )
        config = RobotConfig.from_yaml(config_file)
        self.config = config

        # Create a timer to call the control loop
        self.create_timer(config.control_rate, self.timer_callback)
        self.timer_counter = 0

        # create a subscriber to the joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            "joint_states",
            self.robot_controller.robot.joint_states_callback,
            10,
        )

        # create a subscriber to the imu data
        self.imu_sub = self.create_subscription(
            Imu, "imu", self.robot_controller.robot.imu_callback, 10
        )

        # create a publisher to the joint commands
        self.joint_cmd_pub = self.create_publisher(JointState, "joint_commands", 10)

        # Initialize the robot controller
        self.robot_controller = RobotController(config, self)

    def timer_callback(self):
        self.robot_controller.push_event(RobotEvent.TIMER_EVENT)
        self.timer_counter += 1
        if self.timer_counter % 2.0 / self.robot_controller.control_dt:
            self.robot_controller.push_event(RobotEvent.TIME_OUT_2S)
            


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()

    def signal_handler(sig, frame):
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        while rclpy.ok():
            node.robot_controller.run()
            rclpy.spin_once(node)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
