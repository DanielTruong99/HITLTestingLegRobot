import rclpy
import numpy as np
import sys
import signal
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Joy
from interfaces.msg import KeyInfo

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

        # Initialize the robot controller
        self.robot_controller = RobotController(config, self)

        # Create a timer to call the control loop
        self.create_timer(1.0 / config.control_rate, self.timer_callback)
        self.timer_counter = 0

        # Create a subscriber to the joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            "joint_feedback",
            self.robot_controller.robot.joint_states_callback,
            10,
        )

        # Create a subscriber to the imu data
        self.imu_sub = self.create_subscription(
            Imu, "imu", self.robot_controller.robot.imu_callback, 10
        )

        # Crerate a subscriber to the cmd_vel
        self.joystick_sub = self.create_subscription(
            Joy, "cmd_vel", self.joystick_callback, 10
        )

        # Create a subscriber to the key_info
        self.key_info_sub = self.create_subscription(
            KeyInfo, "key_info", self.key_info_callback, 10
        )

        # Create a publisher to the joint commands
        self.joint_cmd_pub = self.create_publisher(JointState, "joint_cmd", 10)

        # Internal variables
        self.is_on_timer = False

    def key_info_callback(self, msg: KeyInfo):
        """
        Callback function for the key_info subscriber.

        Args:
            msg (KeyInfo): the message
        """
        pass

    def joystick_callback(self, msg: Joy):
        """
        Callback function for the cmd_vel subscriber.
        
        Args:
            msg (Twist): the message
        """
        axes = msg.axes

        left_x = -axes[0]
        left_y = axes[1]
        right_x = -axes[3]
        v_x = left_x * self.robot_controller.config.max_linear_velocity
        v_y = left_y * self.robot_controller.config.max_linear_velocity
        w = right_x * self.robot_controller.config.max_angular_velocity

        self.robot_controller.set_command(np.array([v_x, v_y, w]))

    def timer_callback(self):
        self.robot_controller.push_event(RobotEvent.TIMER_EVENT)
        if self.is_on_timer is False:
            return
        
        self.timer_counter += 1
        if self.timer_counter % 2.0 / self.robot_controller.current_controller.control_dt == 0:
            self.robot_controller.push_event(RobotEvent.TIME_OUT_2S)

    def reset_timer(self):
        self.is_on_timer = False
        self.timer_counter = 0
    
    def start_timer(self):
        self.is_on_timer = True
        self.timer_counter = 0
            


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
