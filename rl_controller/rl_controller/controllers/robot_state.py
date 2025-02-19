import numpy as np

from .robot_config import RobotConfig


class LegState(object):
    def __init__(self, config: RobotConfig) -> None:
        """
        Initialize the leg state object.
        """
        self.position = np.zeros(config.action_dim)
        self.velocity = np.zeros(config.action_dim)


class BaseState(object):
    def __init__(self, config: RobotConfig) -> None:
        """
        Initialize the base state object.
        """
        self.vB = np.zeros(3)
        self.wB = np.zeros(3)
        self.orientation = np.zeros(4)
