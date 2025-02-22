import torch
from torch.jit import ScriptModule
import numpy as np
import rclpy

from .robot import Robot
from .robot_config import RobotConfig


class PolicyController(object):
    def __init__(self, config: RobotConfig, robot: Robot) -> None:
        """
        Import the policy, initialize process variables.

        Args:
            config (Config): the configuration object
            robot (Robot): the robot object interface with the state of the robot
        """
        self.config = config
        self.robot = robot
        self.load_policy(self.config.policy_file_path)

        # Initialize process variables
        self._previous_action = np.zeros(self.config.action_dim)
        self._policy_counter = 0
        self._control_dt = 1.0 / self.config.control_rate

    @property
    def control_dt(self) -> float:
        """
        Get the control time step.

        Returns:
            float: the control time step
        """
        return self._control_dt

    def forward(self, command: np.ndarray) -> np.ndarray:
        """
        Forward the policy to get the action.

        Args:
            command (np.ndarray): the action command (v_x, v_y, w_z)

        Returns:
            np.ndarray: the joint cmds
        """
        observation = self._compute_observation(command)
        action = self._compute_action(observation)

        action = action.cpu().numpy()
        self._previous_action = action
        self._policy_counter += 1
        return action

    def load_policy(self, policy_file_path: str) -> None:
        """
        Load the policy from the policy file.

        Args:
            policy_file_path (str): the policy file path

        """
        try:
            self.policy: ScriptModule = torch.jit.load(policy_file_path)
            self.policy.eval()
            self.policy.to(self.config.device)
            rclpy.logging._root_logger.info(f"Policy loaded from {policy_file_path}")

        except Exception as e:
            rclpy.logging._root_logger.error(
                f"Failed to load policy from {policy_file_path}: {e}"
            )

    def reset(self) -> None:
        """
        Reset the controller.
        """
        self._previous_action = np.zeros(self.config.action_dim)
        self._policy_counter = 0

    def _compute_action(self, observation: np.ndarray) -> np.ndarray:
        """
        Compute the action from the policy.

        Args:
            observation (np.ndarray): the observation

        Returns:
            np.ndarray: the action
        """
        with torch.no_grad():
            observation = (
                torch.from_numpy(observation).view(1, -1).float().to(self.config.device)
            )
            action = self.policy(observation).detach().view(-1).numpy()

        return action

    def _compute_observation(self, command: np.ndarray) -> np.ndarray:
        """
        Compute the observation from the robot state and the previous action.

        Args:
            command (np.ndarray): the action command (v_x, v_y, w_z)

        Returns:
            np.ndarray: the observation [w, projected_g, command, q, q_dot, previous_action, phase]
        """
        observation = np.zeros(self.config.observation_dim)
        
        period = 0.8
        offset = 0.5
        phase = (self._policy_counter * self._control_dt) % period / period
        sin_phase = np.sin(2 * np.pi * phase)
        cos_phase = np.cos(2 * np.pi * phase)

        num_actions = self.config.action_dim
        observation[:3] = self.robot.wB
        observation[3:6] = self.robot.projected_g
        observation[6:9] = command
        observation[9 : 9 + num_actions] = self.robot.joint_positions
        observation[9 + num_actions : 9 + 2 * num_actions] = self.robot.joint_velocities
        observation[9 + 2 * num_actions : 9 + 3 * num_actions] = self._previous_action
        observation[9 + 3 * num_actions] = sin_phase
        observation[9 + 3 * num_actions + 1] = cos_phase
        return observation
