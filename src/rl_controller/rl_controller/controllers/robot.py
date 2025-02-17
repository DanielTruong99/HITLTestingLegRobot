import rclpy
import numpy as np
from sensor_msgs.msg import JointState, Imu

from .utilities import get_gravity_orientation
from .robot_state import LegState, BaseState
from .robot_config import Config

class Robot(object):
    def __init__(self, config: Config) -> None:
        """
        Initialize the robot object.
        """
        self.leg_states = LegState(config)
        self.base_state = BaseState(config)
    
    @property
    def vB(self):
        """
        Retrieve the velocity of the robot's base.
        
        Returns:
            numpy.ndarray: The velocity vector of the robot's base.
        """
        return self.base_state.vB

    @property
    def wB(self):
        """
        Retrieve the angular velocity of the robot's base.
        
        Returns:
            numpy.ndarray: The angular velocity vector of the robot's base.
        """
        return self.base_state.wB

    @property
    def orientation(self):
        """
        Retrieve the orientation of the robot's base.
        
        Returns:
            numpy.ndarray: The orientation quaternion of the robot's base.
        """
        return self.base_state.orientation

    @property
    def projected_g(self):
        """
        Retrieve the projected gravity vector by rotating the g into base frame.

        Returns:
            numpy.ndarray: The projected gravity vector.
        """
        return get_gravity_orientation(self.orientation)

    @property
    def joint_positions(self):
        """
        Retrieve the joint positions of the robot.
        
        Returns:
            numpy.ndarray: The joint positions.
        """
        return self.leg_states.position

    @property
    def joint_velocities(self):
        """
        Retrieve the joint velocities of the robot.
        
        Returns:
            numpy.ndarray: The joint velocities.
        """
        return self.leg_states.velocity

    def joint_states_callback(self, msg: JointState) -> None:
        """
        Callback for the ros joint states subscriber.

        Args:
            msg (JointState): message from the joint states topic
        """
        # Cache the joint positions and velocities
        position = np.array(msg.position)
        velocity = np.array(msg.velocity)

        # Check is finite values and update the joint states
        self.leg_states.position = position if np.all(np.isfinite(position)) else self.leg_states.position
        self.leg_states.velocity = velocity if np.all(np.isfinite(velocity)) else self.leg_states.velocity

    def imu_callback(self, msg):
        """
        Callback for the ros imu subscriber.

        Args:
            msg (Imu): message from the imu topic
        """
        # Cache the orientation, angular velocity
        orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # Check is finite values and update the base states
        self.base_state.orientation = orientation if np.all(np.isfinite(orientation)) else self.base_state.orientation
        self.base_state.wB = angular_velocity if np.all(np.isfinite(angular_velocity)) else self.base_state.wB