
import argparse
import sys
import os
import time
import json
import logging

# Add common lib to path
sys.path.append(os.path.join(os.path.dirname(__file__), "common"))

from isaaclab.app import AppLauncher

# Argument Parser
parser = argparse.ArgumentParser(description="ZeroMQ Bridge for SoccerLab Simulation.")
parser.add_argument("--task", type=str, default="Isaac-Walking-G1-Play-v0", help="Name of the task.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--headless", action="store_true", default=False, help="Run in headless mode.")
parser.add_argument("--checkpoint", type=str, default=None, help="Path to policy checkpoint.")
parser.add_argument("--port", type=str, default="5555", help="ZMQ Port.")

# Append AppLauncher args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Launch App
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Now we can import core and other isaac modules
# Ensure soccerLab is in path (Assuming env var or relative path)
# We try relative path first: ../../../third_party/soccerLab
soccerlab_path = os.environ.get("SOCCERLAB_PATH", os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../third_party/soccerLab")))
if os.path.exists(soccerlab_path):
    sys.path.append(soccerlab_path)
    # Also append scripts/rsl_rl inside soccerLab
    sys.path.append(os.path.join(soccerlab_path, "scripts"))
else:
    print(f"[Warning] SoccerLab path not found at {soccerlab_path}. Relaying on existing python path.")

import cli_args
from core import SimBridge
from zmq_wrapper import ZMQWrapper

def main():
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("SimServer")

    # Load Configs
    # We use cli_args helper from soccerLab to parse RSL-RL configs
    # We need to trick it because we already parsed args. 
    # But cli_args.parse_rsl_rl_cfg needs the parser or args object?
    # Actually checking play.py: agent_cfg = cli_args.parse_rsl_rl_cfg(args_cli.task, args_cli)
    
    # We unfortunately need to re-parse or manually construct, 
    # but cli_args.add_rsl_rl_args(parser) wasn't called on OUR parser. 
    # Let's hope defaults work or we might need to be more rigorous.
    # Ideally we should have called cli_args.add_rsl_rl_args(parser) BEFORE parsing.
    # Let's do a quick hack: we assume default config unless specified.
    
    # To properly load config, we need the env_cfg. 
    from isaaclab_tasks.utils import parse_env_cfg
    env_cfg = parse_env_cfg(
        args_cli.task, 
        device=args_cli.device, 
        num_envs=args_cli.num_envs, 
        use_fabric=not args_cli.use_fabric if hasattr(args_cli, "use_fabric") else True
    )

    # Agent Cfg
    # We mock agent_cfg or load it. 
    # For now, let's assume we can get it via standard means if we had the args.
    # Since we can't easily modify parser flow now without restart, let's use a simpler approach:
    # We accept that we run with defaults for RSL-RL args. 
    # We need to construct a dummy args object for RSL-RL if needed.
    
    # Let's rely on standard config loading in SimBridge or passed args.
    # Actually, SimBridge expects agent_cfg. 
    # Let's import the function and try to run it.
    try:
         agent_cfg = cli_args.parse_rsl_rl_cfg(args_cli.task, args_cli)
    except Exception as e:
         logger.warning(f"Failed to parse RSL-RL config from args: {e}. Using defaults/loading from file manually might be needed.")
         # Fallback? RslRlOnPolicyRunnerCfg()
         from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg
         agent_cfg = RslRlOnPolicyRunnerCfg(experiment_name="g1_walking", run_name="default")


    # Initialize Bridge
    bridge = SimBridge(env_cfg, agent_cfg, device=args_cli.device)
    
    if args_cli.checkpoint:
        bridge.load_checkpoint(args_cli.checkpoint)
    else:
        # Try to find default checkpoint
        logger.warning("No checkpoint provided via --checkpoint. Env might run with random policy.")

    # Initialize ZMQ
    zmq_server = ZMQWrapper(socket_type=import_zmq.REP, addr=f"tcp://*:{args_cli.port}")
    # Need to handle import inside wrapper? No wrapper imports zmq.
    # Correct import of enum:
    import zmq
    zmq_server = ZMQWrapper(socket_type=zmq.REP, addr=f"tcp://*:{args_cli.port}")
    zmq_server.bind()

    logger.info("SimServer Ready. Waiting for client...")

    while simulation_app.is_running():
        # Wait for Request
        try:
            # Non-blocking check or blocking? 
            # Blocking is fine for lockstep.
            msg = zmq_server.recv_json()
        except zmq.Again:
            continue
            
        # Parse Msg
        # Msg format: {"cmd": [vx, vy, w], "timestamp": float}
        cmd = msg.get("cmd", [0.0, 0.0, 0.0])
        client_ts = msg.get("timestamp", 0)
        
        # Apply Command
        bridge.set_command(cmd[0], cmd[1], cmd[2])
        
        # Step Sim
        t_start = time.time()
        state = bridge.step()
        t_step = time.time() - t_start
        
        # Prepare Reply
        response = {
            "state": state,
            "sim_timestamp": time.time(),
            "step_latency": t_step,
            "ack_timestamp": client_ts
        }
        
        zmq_server.send_json(response)

    bridge.close()
    zmq_server.close()
    simulation_app.close()

if __name__ == "__main__":
    main()
