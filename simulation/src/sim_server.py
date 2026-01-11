
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
parser.add_argument("--task", type=str, default="Loco-G1-Velocity", help="Name of the task.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")

parser.add_argument("--checkpoint", type=str, default=None, help="Path to policy checkpoint.")
parser.add_argument("--port", type=str, default="5555", help="ZMQ Port.")
parser.add_argument("--webview", action="store_true", default=False, help="Enable web viewer.")

# Unified Soccer Args
DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "match_config.json")
parser.add_argument("--config_file", type=str, default=DEFAULT_CONFIG, help="Path to match configuration JSON file.")
# CLI args as fallback/quick setup
parser.add_argument("--num_robots", type=int, default=1, help="Number of robots per team (default 1).")
parser.add_argument("--robot_type", type=str, default="g1", help="Robot type (g1/k1).") 
parser.add_argument("--field_length", type=float, default=9.0, help="Field length.")
parser.add_argument("--field_width", type=float, default=6.0, help="Field width.")

# Append AppLauncher args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# FORCE enable cameras if webview is on, otherwise headless rendering fails
if args_cli.webview:
    args_cli.enable_cameras = True

# Prepare Match Config
match_config = {}
if args_cli.config_file and os.path.exists(args_cli.config_file):
    print(f"[SimServer] Loading match config from {args_cli.config_file}")
    with open(args_cli.config_file, 'r') as f:
        match_config = json.load(f)
else:
    # Construct default config from CLI args
    match_config = {
        "field": {
            "length": args_cli.field_length,
            "width": args_cli.field_width
        },
        "teams": {
            "red": {
                "count": args_cli.num_robots,
                "robot_type": args_cli.robot_type,
                # Dynamic spawn positions will be handled in env_cfg if not present here
            },
            "blue": {
                "count": args_cli.num_robots,
                "robot_type": args_cli.robot_type
            }
        }
    }

# Export env vars for unified config
os.environ["SOCCER_MATCH_CONFIG"] = json.dumps(match_config)
# also keep legacy env vars for compatibility if needed, but main logic should use config
os.environ["SOCCER_NUM_ROBOTS"] = str(args_cli.num_robots)
os.environ["SOCCER_ROBOT_TYPE"] = str(args_cli.robot_type)

# Launch App
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Now we can import core and other isaac modules
# Ensure soccerLab is in path (Assuming env var or relative path)
# We try relative path first: ../../../third_party/soccerLab
soccerlab_path = os.environ.get("SOCCERLAB_PATH", os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../third_party/soccerLab")))
if os.path.exists(soccerlab_path):
    sys.path.append(soccerlab_path)
    # Append source to access soccerTask package
    # NOTE: The structure is source/soccerTask/soccerTask/__init__.py
    # If we add .../source, we find source/soccerTask (outer folder with setup.py).
    # This outer folder is NOT a package (no __init__.py).
    # We must add .../source/soccerTask to path to import the inner soccerTask package.
    sys.path.append(os.path.join(soccerlab_path, "source", "soccerTask"))
    
    # We also need to add 'source' itself to path to find 'robotlib' (which is checked out to source/robotlib)
    sys.path.append(os.path.join(soccerlab_path, "source"))

    # Also append scripts/rsl_rl inside soccerLab
    sys.path.append(os.path.join(soccerlab_path, "scripts"))
    sys.path.insert(0, os.path.join(soccerlab_path, "scripts", "rsl_rl"))
else:
    print(f"[Warning] SoccerLab path not found at {soccerlab_path}. Relaying on existing python path.")

# Import cli_args from rsl_rl
# Note: sys.path includes soccerLab/scripts, so we can import rsl_rl.cli_args or just rsl_rl if scripts is in path?
# sim_server.py added `os.path.join(soccerlab_path, "scripts")` to sys.path.
# And cli_args.py is in `soccerLab/scripts/rsl_rl/cli_args.py`.
# So inside scripts, there is a package `rsl_rl`? No, `rsl_rl` is a folder.
# Is `rsl_rl` inside scripts a package (has __init__.py)?
# Let's import it as `from rsl_rl import cli_args` assuming scripts is in path.
try:
    import cli_args # Try direct import from inserted path
except ImportError:
     # Fallback if rsl_rl is not a package in scripts
    from rsl_rl import cli_args # Try standard way

try:
    import soccerTask
    import soccerTask.train # This triggers registration in soccerTask/train/__init__.py
    import soccerTask.train.locomotion.velocity.g1 # Explicitly import G1 to ensure registration (since velocity/__init__.py is empty)
    import soccerTask.soccer # Register soccer tasks
except ImportError as e:
    print(f"[Warning] Could not import soccerTask: {e}")
    import sys
    print(f"DEBUG: sys.path: {sys.path}")
    print(f"DEBUG: soccerlab_path: {soccerlab_path}")


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

    # PATCH: Force correct hidden dimensions for G1 if they don't match checkpoint expectation
    # Handle potentially missing policy config from fallback
    is_policy_missing = (not hasattr(agent_cfg, "policy")) or (type(agent_cfg.policy).__name__ == "_MISSING_TYPE")
    
    if is_policy_missing:
         from isaaclab_rl.rsl_rl import RslRlPpoActorCriticCfg
         agent_cfg.policy = RslRlPpoActorCriticCfg(
             init_noise_std=1.0,
             actor_hidden_dims=[512, 256, 128],
             critic_hidden_dims=[512, 256, 128],
             activation="elu",
         )

    if hasattr(agent_cfg, "policy"):
         # Handle missing dims
         if type(agent_cfg.policy.actor_hidden_dims).__name__ == "_MISSING_TYPE":
              agent_cfg.policy.actor_hidden_dims = [256, 256, 128]
         if type(agent_cfg.policy.critic_hidden_dims).__name__ == "_MISSING_TYPE":
              agent_cfg.policy.critic_hidden_dims = [256, 256, 128]
         # Handle missing activation
         if not hasattr(agent_cfg.policy, "activation") or type(agent_cfg.policy.activation).__name__ == "_MISSING_TYPE":
              agent_cfg.policy.activation = "elu"

         logger.info(f"Current policy config: actor={agent_cfg.policy.actor_hidden_dims}, critic={agent_cfg.policy.critic_hidden_dims}")
         if args_cli.robot_type == "g1":
              target_dims = [512, 256, 128]
              if agent_cfg.policy.actor_hidden_dims != target_dims:
                   logger.warning(f"Overriding actor hidden dims from {agent_cfg.policy.actor_hidden_dims} to {target_dims}")
                   agent_cfg.policy.actor_hidden_dims = target_dims
              if agent_cfg.policy.critic_hidden_dims != target_dims:
                   logger.warning(f"Overriding critic hidden dims from {agent_cfg.policy.critic_hidden_dims} to {target_dims}")
                   agent_cfg.policy.critic_hidden_dims = target_dims


    # Initialize Bridge
    webview_enabled = args_cli.webview
    if webview_enabled:
        # Try to find labWebView
        # We assume it is in the same workspace root as mos-brain or passed via env?
        # Based on file structure: mos-brain is at /home/charlie/wj_ws/MOS_sim/mos-brain
        # labWebView is at /home/charlie/wj_ws/MOS_sim/labWebView
        
        # Get mos-brain root (../../..) from this file
        mos_brain_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        # One more up for workspace root
        ws_root = os.path.dirname(mos_brain_root)
        
        labwebview_path = os.environ.get("LABWEBVIEW_PATH", os.path.join(ws_root, "labWebView"))
        
        if os.path.exists(labwebview_path):
             sys.path.insert(0, os.path.join(labwebview_path, "source", "labwebView"))
             logger.info(f"Added labWebView to path (priority): {labwebview_path}")
        else:
             logger.warning(f"labWebView not found at {labwebview_path}. Webview might fail.")

    bridge = SimBridge(env_cfg, agent_cfg, args_cli.task, device=args_cli.device, enable_webview=webview_enabled, webview_port=5811)
    
    if args_cli.checkpoint:
        bridge.load_checkpoint(args_cli.checkpoint)
    else:
        # Try to find default checkpoint
        default_ckpt = None
        if args_cli.robot_type == "g1":
            #  default_ckpt = os.path.join(soccerlab_path, "data/ckpts/g1/g1_29d_loco_walk.pt")
            # Use the original model checkpoint (not JIT) - the adaptation system handles dimension conversion
            default_ckpt = os.path.join(soccerlab_path, "logs/rsl_rl/g1_loco/2026-01-08_16-18-29_cliped_with_lin/exported/policy.pt")
        
        if default_ckpt and os.path.exists(default_ckpt):
             logger.info(f"Loading default checkpoint for {args_cli.robot_type}: {default_ckpt}")
             bridge.load_checkpoint(default_ckpt)
        else:
             logger.warning("No checkpoint provided via --checkpoint. Env might run with random policy.")

    # Initialize ZMQ
    import zmq
    zmq_server = ZMQWrapper(socket_type=zmq.REP, addr=f"tcp://*:{args_cli.port}")
    zmq_server.bind()

    logger.info("SimServer Ready. Waiting for client...")

    logger.info("SimServer Ready. Waiting for client...")
    logger.info(f"simulation_app.is_running() = {simulation_app.is_running()}")

    try:
        running = True
        while running:
            # Check if simulation app is still valid
            if not simulation_app.is_running():
                logger.warning("simulation_app.is_running() returned False, but continuing...")
                # Don't break - in headless mode this can return False even when functional
            
            # Wait for Request
            try:
                # Use non-blocking receive if webview is enabled to allow rendering
                flags = zmq.NOBLOCK if webview_enabled else 0
                msg = zmq_server.socket.recv_json(flags=flags)
                
                # Parse Msg
                # Msg format: {"cmd": [vx, vy, w], "timestamp": float, "id": int}
                cmd = msg.get("cmd", [0.0, 0.0, 0.0])
                client_ts = msg.get("timestamp", 0)
                robot_id = msg.get("id", 0)
                
                # DEBUG: Log non-zero commands
                if any(abs(c) > 0.001 for c in cmd):
                    logger.info(f"[SimServer] Received ZMQ CMD for Robot {robot_id}: {cmd}")

                # Apply Command
                bridge.set_command(cmd[0], cmd[1], cmd[2], robot_id=robot_id)
                
                # Step Sim
                t_start = time.time()
                state = bridge.step()
                t_step = time.time() - t_start

                # Real-time sync (Default)
                # Get dt from env (default to 0.02 if not present)
                sim_dt = getattr(bridge.env.unwrapped, "step_dt", 0.02) 
                sleep_time = sim_dt - t_step
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                # Prepare Reply
                response = {
                    "state": state,
                    "sim_timestamp": time.time(),
                    "step_latency": t_step,
                    "ack_timestamp": client_ts
                }
                
                zmq_server.send_json(response)
                
            except zmq.Again:
                # No message received
                if webview_enabled:
                     # Auto-step to keep webview alive
                     bridge.step()
                continue
            except Exception as e:
                logger.error(f"Critical error in loop: {e}")
                import traceback
                traceback.print_exc()
                break # Exit loop on critical error to prevent infinite spam and allow shutdown
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received, shutting down...")
    finally:
        bridge.close()
        zmq_server.close()
        simulation_app.close()



if __name__ == "__main__":
    main()
