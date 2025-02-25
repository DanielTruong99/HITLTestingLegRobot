import rclpy
import numpy as np
import inspect
from sensor_msgs.msg import JointState, Imu

from .utilities import get_gravity_orientation
from .robot_state import LegState, BaseState
from .robot_config import RobotConfig


class Robot(object):
    def __init__(self, config: RobotConfig) -> None:
        """
        Initialize the robot object.
        """
        self.leg_states = LegState(config)
        self.base_state = BaseState(config)

        # self.callback_flags = {
        #     "joint_states_callback": False,
        #     "imu_callback": False,
        # }
        self.callback_flags = {
            key: False
            for key, _ in inspect.getmembers(self, predicate=inspect.ismethod)
            if key.endswith("_callback")
        }

    @property
    def is_ready(self) -> bool:
        """
        Check if the sensor datas has came.

        Returns:
            bool: True if the robot is ready
        """
        return all(self.callback_flags.values())

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
        return self.orientation

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

    def is_safe(self) -> bool:
        """
        Check if the robot is in a safe state.

        Returns:
            bool: True if the robot is in a safe state.
        """
        is_safe = True

        # Check roll and pitch angles
        is_safe = is_safe and bool(np.abs(self.projected_g[0]) < 0.7)
        is_safe = is_safe and bool(np.abs(self.projected_g[1]) < 0.7)

        return is_safe

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
        self.leg_states.position = (
            position if np.all(np.isfinite(position)) else self.leg_states.position
        )
        self.leg_states.velocity = (
            velocity if np.all(np.isfinite(velocity)) else self.leg_states.velocity
        )

        # Set the callback flag to True
        if self.callback_flags["joint_states_callback"] is False:
            self.callback_flags["joint_states_callback"] = True
            

    def imu_callback(self, msg: Imu) -> None:
        """
        Callback for the ros imu subscriber.

        Args:
            msg (Imu): message from the imu topic
        """
        # Cache the orientation, angular velocity
        # Temporarily store the projected gravity vector
        # orientation = np.array(
        #     [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        # )
        orientation = np.array(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z]
        )
        angular_velocity = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        )

        # Check is finite values and update the base states
        self.base_state.orientation = (
            orientation
            if np.all(np.isfinite(orientation))
            else self.base_state.orientation
        )
        self.base_state.wB = (
            angular_velocity
            if np.all(np.isfinite(angular_velocity))
            else self.base_state.wB
        )

        # Set the callback flag to True
        if self.callback_flags["imu_callback"] is False:
            self.callback_flags["imu_callback"] = True
