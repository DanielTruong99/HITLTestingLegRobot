import rclpy
from sensor_msgs.msg import JointState
from dataclasses import dataclass
from .robot_controller import RobotController
from .robot import Robot
from ..finite_state_machine import fsm as state_machine

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
            # Move to default joint in damping mode
            self.controller.node_handler.joint_cmd_pub.publish(self.controller._joint_cmd_damping_msg)

            # Set the timer_counter to 0
            self.controller.node_handler.timer_counter = 0
            rclpy.logging._root_logger.info("Robot in configuration state")
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
            
            # Publish the joint commands, velocity field is used to set kp and kd
            joint_cmd_msg = JointState()
            for joint_cmd in joint_cmds:
                joint_cmd_msg.position.append(joint_cmd)
            joint_cmd_msg.velocity = self.robot_controller.config.kps + self.robot_controller.config.kds
            self.node_handler.joint_cmd_pub.publish(joint_cmd_msg)

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