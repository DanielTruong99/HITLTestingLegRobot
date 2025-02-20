import numpy as np
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ..controllers import state_machine
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

    def run(self) -> None:
        """
        Run the robot controller.
        """
        if len(self.event_queue) > 0:
            event = self.event_queue.pop(0)
            self.state_machine.dispatch(event)

    def push_event(self, event: state_machine.BuiltInEvent) -> None:
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

@dataclass
class RobotEvent(state_machine.BuiltInEvent):
    TIME_OUT_2S = state_machine.BuiltInEvent.USER_SIG
    TIMER_EVENT = state_machine.BuiltInEvent.USER_SIG + 1
 

class RobotFSM(state_machine.FSM):
    def __init__(self, robot: Robot, controller: RobotController) -> None:
        """
        Initialize the robot FSM.

        Args:
            config (RobotConfig): the configuration object
        """
        super().__init__()
        self.robot: Robot = robot
        self.controller = controller

    def initial_state(self, event: RobotEvent) -> state_machine.Status:
        """
        The initial state of the robot FSM.
        
        Args:
            event (RobotEvent): the event

        Returns:
            state_machine.Status: the status
        """
        status = state_machine.Status.IGNORED_STATUS

        if event is RobotEvent.ENTRY_SIG:
            rclpy.logging._root_logger.info("Robot in initial state")
            status = state_machine.Status.TRAN_STATUS

        else:
            if self.robot.is_ready:
                rclpy.logging._root_logger.info("All sensor datas are ready")
                self.transition_to(self.configuration_state)
                status = state_machine.Status.TRAN_STATUS

        return status
    
    def configuration_state(self, event: RobotEvent) -> state_machine.Status:
        """
        The configuration state of the robot FSM.
        
        Args:
            event (RobotEvent): the event
                
        Returns:
            state_machine.Status: the status
        """
        status = state_machine.Status.IGNORED_STATUS

        if event is RobotEvent.ENTRY_SIG:
            rclpy.logging._root_logger.info("Robot in configuration state")
            # set pd gain 
            # Move to default joint

            # Set the timer_counter to 0
            self.controller.node_handler.timer_counter = 0
            status = state_machine.Status.HANDLED_STATUS

        elif event is RobotEvent.TIME_OUT_2S:
            self.transition_to(self.running_state)
            status = state_machine.Status.TRAN_STATUS


        return status
    
    def running_state(self, event: RobotEvent) -> state_machine.Status:
        """
        The running state of the robot FSM.
        
        Args:
            event (RobotEvent): the event
                
        Returns:
            state_machine.Status: the status
        """
        status = state_machine.Status.IGNORED_STATUS

        # 50Hz
        if event is RobotEvent.TIMER_EVENT:
            # Compute the joint commands
            joint_cmds = self.controller.compute().tolist()
            
            # Publish the joint commands
            joint_state = JointState()
            for joint_cmd in joint_cmds:
                joint_state.position.append(joint_cmd)
            self.node_handler.joint_cmd_pub.publish(joint_state)

            status = state_machine.Status.HANDLED_STATUS
        
        # Default
        else:
            # check safety
            status = state_machine.Status.HANDLED_STATUS


        return status

    def error_state(self, event: RobotEvent) -> state_machine.Status:
        """
        The error state of the robot FSM.
        
        Args:
            event (RobotEvent): the event
                
        Returns:
            state_machine.Status: the status
        """
        status = state_machine.Status.IGNORED_STATUS

        if event is RobotEvent.ENTRY_SIG:
            # stop the robot
            # log error
            status = state_machine.Status.HANDLED_STATUS

        return status


