
import argparse
from isaaclab.app import AppLauncher

# Create generic parser
parser = argparse.ArgumentParser(description="Debug Env")
# Add AppLauncher args
AppLauncher.add_app_launcher_args(parser)
# Parse
args_cli = parser.parse_args()

# Launch App
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Now imports
import gymnasium as gym
import torch
import sys
import os
# Register user extensions
# Assuming soccerLab and soccerTask need to be imported to register the task
# We might need to add paths if they are not installed packages
sys.path.append("/home/charlie/wj_ws/MOS_sim/soccerLab/source")
sys.path.append("/home/charlie/wj_ws/MOS_sim/soccerLab/source/soccerTask")

# Try importing the registration module
# Since we don't know exactly where it is, we hoping 'import soccerTask.soccerTask' or similar triggers it
# Based on file structure: soccerLab/source/soccerTask/soccerTask/__init__.py likely registers

import soccerTask.soccerTask # This likely registers the tasks

from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
from isaaclab.envs import ManagerBasedRLEnv # Check this too

# Manual Args
TASK_NAME = "Loco-Unified-Soccer"
NUM_ENVS = 2

def main():
    print(f"Creating env for {TASK_NAME}")
    
    # We use gym.make directly as that uses the registry
    try:
        env = gym.make(TASK_NAME, cfg={"scene": {"num_envs": NUM_ENVS}}, render_mode=None)
    except Exception as e:
        print(f"Failed to gym.make: {e}")
        return

    print(f"Env Class: {type(env)}")
    print(f"Env Unwrapped Class: {type(env.unwrapped)}")
    print(f"Is DirectMARLEnv? {isinstance(env.unwrapped, DirectMARLEnv)}")
    print(f"Is ManagerBasedRLEnv? {isinstance(env.unwrapped, ManagerBasedRLEnv)}")

    obs, info = env.reset()
    
    # Handle dict obs matches
    if isinstance(obs, dict):
        print(f"Initial Obs is dict. Keys: {obs.keys()}")
        # Check policy entry
        if "policy" in obs:
            print(f"Obs['policy'] Shape: {obs['policy'].shape}")
        else:
             first_val = next(iter(obs.values()))
             print(f"First Val Shape: {first_val.shape}")
    else:
        print(f"Initial Obs Shape: {obs.shape}")

    # Check Multi-Agent Wrapper behavior
    if isinstance(env.unwrapped, DirectMARLEnv):
        print("Wrapping with multi_agent_to_single_agent...")
        env = multi_agent_to_single_agent(env)
        print(f"Wrapped Env Class: {type(env)}")
        obs, _ = env.reset()
        if isinstance(obs, dict) and "policy" in obs:
             print(f"Wrapped Obs['policy'] Shape: {obs['policy'].shape}")
        else:
             print(f"Wrapped Obs Shape: {obs.shape}")
    
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()
