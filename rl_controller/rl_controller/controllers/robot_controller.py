import numpy as np
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .state_machine import RobotFSM, RobotEvent
from .robot import Robot
from .robot_config import RobotConfig
from .policy_controller import PolicyController

class RobotController(object):
    def __init__(self, config: RobotConfig, node_handler: Node) -> None:
        """
        Initialize the robot controller, robot object for state handling.

        Args:
            config (RobotConfig): the configuration object
        """
        self.node_handler: Node = node_handler
        self.config = config

        # Initialize the robot object for state handling
        self.robot: Robot = Robot(config)

        # Initialize the policy controller
        self.current_controller = PolicyController(config, self.robot)

        # Initialize the state machine
        self.state_machine = RobotFSM(self.robot, self)
        self.event_queue = []
        self._max_event_queue_size = 20
        self.push_event(RobotEvent.ENTRY_SIG)

        # Initialize the command, action
        self._command = np.zeros(3)
        self._command[0] = 0.0

        # Initialize the start moving to default flag
        self._is_start_moving_to_default = False
        self._moving_to_default_duration = 2.0
        self._moving_to_defaul_counter = 0
        self._is_done_moving_to_default_position = False
        self._init_joint_positions = {
            "stand": np.array(config.standing_joint_positions),
            "kneel": np.array(config.kneeling_joint_positions),
        }

        # Initialize some special messages 
        self._joint_cmd_stop_msg = JointState()
        self._joint_cmd_stop_msg.position = [0.0] * config.action_dim
        kps = [0.0] * config.action_dim
        kds = [0.0] * config.action_dim
        self._joint_cmd_stop_msg.velocity = kps + kds
        
        self._joint_cmd_damping_msg = JointState()
        self._joint_cmd_damping_msg.position = [0.0] * config.action_dim
        kps = [1000.0] * config.action_dim
        kds = [10.0] * config.action_dim
        self._joint_cmd_damping_msg.velocity = kps + kds

    def stop(self) -> None:
        """
        Stop the robot controller.
        """
        self.node_handler.joint_cmd_pub.publish(self._joint_cmd_stop_msg)

    def run(self) -> None:
        """
        Run the robot controller.
        """
        if len(self.event_queue) > 0:
            event = self.event_queue.pop(0)
        else:
            event = RobotEvent.DEFAULT_SIG    
        self.state_machine.dispatch(event)

    def push_event(self, event: RobotEvent) -> None:
        """
        Push the event to the event queue.

        Args:
            event (state_machine.BuiltInEvent): the event to push
        """
        if len(self.event_queue) < self._max_event_queue_size:
            self.event_queue.append(event)

    def is_ready(self) -> bool:
        """
        Check if the sensor datas has came.

        Returns:
            bool: True if the robot is ready
        """
        return self.robot.is_ready

    def set_command(self, command: np.ndarray) -> None:
        """
        Set the command to the robot.

        Args:
            command (np.ndarray): the command to set
        """
        self._command = command

    def is_start_moving_to_default(self) -> bool:
        """
        Check if the robot is start moving to the default position.

        Returns:
            bool: True if the robot is start moving to the default position
        """
        return self._is_start_moving_to_default

    def start_moving_to_default(self, duration) -> None:
        """
        Start moving to the default position.

        Args:
            duration (float): the duration to move to the default, in seconds
        """
        self._is_start_moving_to_default = True
        self._moving_to_default_duration = duration

    def move_to_default_position(self, init_pose_type: str) -> None:
        """
        Move to the default position.

        Args:
            init_pose_type (str): the initial pose type, "stand" or "kneel"
        """
        self._moving_to_defaul_counter += 1
        alpha = self._moving_to_defaul_counter / (self._moving_to_default_duration * self.config.control_rate)
        if alpha >= 1.0:
            self._is_start_moving_to_default = False
            self._moving_to_defaul_counter = 0
            self._is_done_moving_to_default_position = True
            return

        target_joint_positions = (1 - alpha) * self.robot.joint_positions + alpha * self._init_joint_positions.get(init_pose_type, "stand")
        self._joint_cmd_damping_msg.header.stamp = self.node_handler.get_clock().now().to_msg()
        self._joint_cmd_damping_msg.position = target_joint_positions.tolist()
        self.node_handler.joint_cmd_pub.publish(self._joint_cmd_damping_msg)


    def is_done_moving_to_default_position(self) -> bool:
        """
        Check if the robot is done moving to the default position.

        Returns:
            bool: True if the robot is done moving to the default position
        """
        return self._is_done_moving_to_default_position

    def compute(self) -> np.ndarray:
        """
        Compute the action from the policy controller.

        Args:
            command (np.ndarray): the action command (v_x, v_y, w_z)

        Returns:
            np.ndarray: the joint cmds
        """
        return self.current_controller.forward(self._command)
