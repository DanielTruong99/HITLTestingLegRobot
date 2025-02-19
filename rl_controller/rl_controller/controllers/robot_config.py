import yaml
from dataclasses import dataclass
from typing import List, Dict, Any

@dataclass
class RobotConfig:
    action_dim: int
    observation_dim: int
    device: str
    policy_file_path: str

    @classmethod
    def from_yaml(cls, config_file_path: str) -> 'RobotConfig':
        """
        Load the robot config from the yaml file.

        Args:
            config_file_path (str): the config file path

        Returns:
            RobotConfig: the robot config
        """
        with open(config_file_path, 'r') as f:
            config = yaml.safe_load(f)
        
        return cls(**config)