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
        self._command[0] = 0.7

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

    def compute(self) -> np.ndarray:
        """
        Compute the action from the policy controller.

        Args:
            command (np.ndarray): the action command (v_x, v_y, w_z)

        Returns:
            np.ndarray: the joint cmds
        """
        return self.current_controller.forward(self._command)
