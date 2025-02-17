import torch
import numpy as np
import rclpy

class PolicyController:
    def __init__(self, config, robot) -> None:
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
            self.policy = torch.jit.load(policy_file_path)
            self.policy.eval()
            self.policy.to(self.config.device)
            rclpy.logger._root_logger.info(f"Policy loaded from {policy_file_path}")
        
        except Exception as e:
            rclpy.logger._root_logger.error(f"Failed to load policy from {policy_file_path}: {e}")

    def _compute_action(self, observation: np.ndarray) -> np.ndarray:
        """
        Compute the action from the policy.

        Args:
            observation (np.ndarray): the observation

        Returns:
            np.ndarray: the action
        """
        with torch.no_grad():
            observation = torch.from_numpy(observation).view(1, -1).float().to(self.config.device)
            action = self.policy(observation).detach().view(-1).numpy()
    
        return action

    def _compute_observation(self, command: np.ndarray) -> np.ndarray:
        """
        Compute the observation from the robot state and the previous action.

        Args:
            command (np.ndarray): the action command (v_x, v_y, w_z)

        Returns:
            np.ndarray: the observation [v, w, projected_g, command, q, q_dot, previous_action]
        """
        observation = np.zeros(self.config.observation_dim)
        v_B = self.robot.vB
        w_B = self.robot.wB
        projected_g = self.robot.projected_g
        q = self.robot.joint_positions
        q_dot = self.robot.joint_velocities
        num_actions = self.config.action_dim

        observation[:3] = v_B
        observation[3:6] = w_B
        observation[6:9] = projected_g
        observation[9:12] = command
        observation[12 : 12 + num_actions] = q
        observation[12 + num_actions : 12 + 2*num_actions] = q_dot
        observation[12 + 2*num_actions : 12 + 3*num_actions] = self._previous_action
        return observation

            