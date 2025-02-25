import rclpy
from sensor_msgs.msg import JointState
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any
if TYPE_CHECKING:
    from .robot_controller import RobotController
from .robot import Robot
from ..finite_state_machine import fsm as state_machine

@dataclass
class RobotEvent(state_machine.BuiltInEvent):
    TIME_OUT_2S = state_machine.BuiltInEvent.USER_SIG
    TIMER_EVENT = state_machine.BuiltInEvent.USER_SIG + 1
 

class RobotFSM(state_machine.FSM):
    def __init__(self, robot: Robot, controller: Any) -> None:
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
            status = state_machine.Status.HANDLED_STATUS

        elif event is RobotEvent.TIMER_EVENT:
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
            # Start the robot controller to move to default joint position
            self.controller.start_moving_to_default_position()
            rclpy.logging._root_logger.info("Robot in configuration state")
            status = state_machine.Status.HANDLED_STATUS

        elif event is RobotEvent.TIMER_EVENT:
            if self.controller.robot_controller.is_start_moving_to_default():
                # Robot move to its default joint position
                self.controller.robot_controller.move_to_default_position("kneel")

                # Check if the robot is done moving to default position
                # Start the timer for waiting 2 seconds
                if self.controller.robot_controller.is_done_moving_to_default_position():
                    rclpy.logging._root_logger.info("Robot is done moving to default position")
                    rclpy.logging._root_logger.info("Robot is waiting for 2 seconds ...")
                    self.controller.node_handler.start_timer()
            status = state_machine.Status.HANDLED_STATUS

        elif event is RobotEvent.TIME_OUT_2S:
            # Reset the timer after moving to default position
            self.controller.node_handler.reset_timer()
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
            
            # Publish the joint commands, velocity field is used to set kp and kd
            joint_cmd_msg = JointState()
            joint_cmd_msg.header.stamp = self.controller.node_handler.get_clock().now().to_msg()
            for joint_cmd in joint_cmds:
                joint_cmd_msg.position.append(joint_cmd)
            joint_cmd_msg.velocity = self.controller.current_controller.config.kps + self.controller.current_controller.config.kds
            self.controller.node_handler.joint_cmd_pub.publish(joint_cmd_msg)

            status = state_machine.Status.HANDLED_STATUS
        
        elif event is RobotEvent.ENTRY_SIG:
            rclpy.logging._root_logger.info("Robot in running state")
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
            # Stop the all the motors
            self.controller.stop()
            
            # Reset the robot controller
            self.controller.current_controller.reset()
            
            rclpy.logging._root_logger.error("Robot in error state")
            status = state_machine.Status.HANDLED_STATUS

        return status