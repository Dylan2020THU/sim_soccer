
import os
import sys
import torch
import gymnasium as gym

from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, export_policy_as_jit, export_policy_as_onnx
from rsl_rl.runners import OnPolicyRunner

# Add path to query rsl_rl_utils and cli_args
# Assuming this file is in mos-brain/simulation/soccerlab_bridge/core.py
# and soccerLab is at mos-brain/third_party/soccerLab or configured via env
# We'll handle sys.path in sim_server.py/main setup, so we assume successful import here.

class SimBridge:
    def __init__(self, env_cfg, agent_cfg, device="cuda"):
        self.device = device
        self.current_command = torch.tensor([0.0, 0.0, 0.0], device=self.device)
        self.reset_needed = False
        
        # Override velocity commands to read from self.current_command
        # We assume env_cfg has observations.policy.velocity_commands
        if hasattr(env_cfg.observations.policy, "velocity_commands"):
             env_cfg.observations.policy.velocity_commands.func = self._get_velocity_command

        # Create Environment
        self.env = gym.make(agent_cfg.task, cfg=env_cfg, render_mode=None) # No render mode for now, or "rgb_array" if needed?

        # Handle MARL
        if isinstance(self.env.unwrapped, DirectMARLEnv):
            self.env = multi_agent_to_single_agent(self.env)
        
        # RSL-RL Wrapper
        self.env = RslRlVecEnvWrapper(self.env, clip_actions=agent_cfg.clip_actions)
        
        # Load Policy
        # Assume checkpoint path is passed or determined before
        self.ppo_runner = OnPolicyRunner(self.env, agent_cfg.to_dict(), log_dir=None, device=device)
        # Checkpoint loading should be handled by runner.load() called externally or here if path provided
        
        self.obs, _ = self.env.get_observations()
        self.policy = None

    def load_checkpoint(self, path):
        print(f"[SimBridge] Loading checkpoint: {path}")
        self.ppo_runner.load(path)
        self.policy = self.ppo_runner.get_inference_policy(device=self.env.unwrapped.device)
        self.policy.eval()

    def _get_velocity_command(self, env):
        # Return current command repeated for num_envs
        # Shape: (num_envs, 3)
        return self.current_command.unsqueeze(0).repeat(env.num_envs, 1)

    def set_command(self, vx, vy, w):
        self.current_command[:] = torch.tensor([vx, vy, w], device=self.device)

    def step(self):
        if self.policy is None:
            raise RuntimeError("Policy not loaded. Call load_checkpoint() first.")

        with torch.inference_mode():
            actions = self.policy(self.obs)
            self.obs, _, _, _ = self.env.step(actions)
            
        return self._extract_state()

    def _extract_state(self):
        # Extract Ground Truth state from env
        # This depends heavily on the specific Env implementation (SoccerEnv?)
        # We need to access the underlying env to get robot state and ball state.
        
        # Unwrap to get to the base env
        base_env = self.env.unwrapped
        
        # Depending on how state is stored in Isaac Lab tasks:
        # Usually base_env.scene["robot"].data.root_pose_w gives world pose
        robot = base_env.scene["robot"]
        robot_pos = robot.data.root_pose_w[0, :3]  # (x, y, z) - taking first env
        robot_quat = robot.data.root_pose_w[0, 3:] # (w, x, y, z)
        
        # Calculate yaw from quat
        # ... (yaw calc logic)
        
        # Ball position
        # Assuming ball is an object in scene named "ball" or similar
        # Need to verify object names in soccerLab task config!
        ball_pos = [0.0, 0.0, 0.0]
        if "ball" in base_env.scene.keys():
             ball = base_env.scene["ball"]
             ball_pos = ball.data.root_pose_w[0, :3].tolist()
        
        state = {
            "robot": {
                "x": float(robot_pos[0]),
                "y": float(robot_pos[1]),
                "theta": self._quat_to_yaw(robot_quat), # Need helper
            },
            "ball": {
                "x": float(ball_pos[0]),
                "y": float(ball_pos[1]),
                "z": float(ball_pos[2])
            }
        }
        return state

    def _quat_to_yaw(self, q):
        # q: (w, x, y, z)
        w, x, y, z = q
        # roll (x-axis rotation)
        # sinr_cosp = 2 * (w * x + y * z)
        # cosr_cosp = 1 - 2 * (x * x + y * y)
        # roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = torch.atan2(siny_cosp, cosy_cosp)
        return float(yaw)

    def close(self):
        self.env.close()
