import numpy as np

from .robot import Robot
from .robot_config import RobotConfig
from .policy_controller import PolicyController

class RobotController(object):
    def __init__(self, config: RobotConfig) -> None:
        """
        Initialize the robot controller, robot object for state handling.
        
        Args:
            config (RobotConfig): the configuration object
        """
        self.robot = Robot(config)
        self.controller = PolicyController(config, self.robot)

    def compute(self, command: np.ndarray) -> np.ndarray:
        """
        Compute the action from the policy controller.

        Args:
            command (np.ndarray): the action command (v_x, v_y, w_z)

        Returns:
            np.ndarray: the joint cmds
        """
        return self.controller.forward(command)