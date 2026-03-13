#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) MOS-Brain Contributors
# SPDX-License-Identifier: GPL-3.0-or-later

# Usage: python3 decider/tools/keyboard_cmd_vel.py --team red --number 0 --port 5555

import argparse
import json
import select
import sys
import termios
import tty
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


MAX_X = 1.5
MAX_Y = 1.0
MAX_Z = 3.0


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def load_red_count() -> int:
    cfg_candidates = [
        PROJECT_ROOT / "simulation/mujoco/assets/config/match_config.json",
        PROJECT_ROOT / "simulation/isaac_sim/config/match_config.json",
    ]
    for cfg in cfg_candidates:
        if cfg.exists():
            with open(cfg, "r", encoding="utf-8") as f:
                data = json.load(f)
            return int(data.get("teams", {}).get("red", {}).get("count", 7))
    return 7


def resolve_robot_id(team: str, number: int) -> int:
    if number < 0:
        raise ValueError("--number must be >= 0")
    red_count = load_red_count()
    if team == "blue":
        return red_count + number
    return number


def read_key(timeout_sec: float):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if not rlist:
        return None
    return sys.stdin.read(1)


def print_help() -> None:
    print("")
    print("Keyboard control:")
    print("  w/s : x + / -")
    print("  a/d : y + / -")
    print("  q/e : z + / -")
    print("  space: stop (x=y=z=0)")
    print("  h : print this help")
    print("  x : exit")
    print("")


def main() -> None:
    parser = argparse.ArgumentParser(description="Keyboard cmd_vel teleop via decider SimClient")
    parser.add_argument("--ip", default="127.0.0.1", help="ZMQ server ip")
    parser.add_argument("--port", type=int, default=5555, help="ZMQ server port")
    parser.add_argument("--team", choices=["red", "blue"], default="red", help="Team color")
    parser.add_argument("--number", type=int, default=0, help="Player number in team")
    parser.add_argument("--rate", type=float, default=20.0, help="Send rate (Hz)")
    parser.add_argument("--step-x", type=float, default=0.2, help="x increment per key press")
    parser.add_argument("--step-y", type=float, default=0.1, help="y increment per key press")
    parser.add_argument("--step-z", type=float, default=0.3, help="z increment per key press")
    args = parser.parse_args()

    if args.rate <= 0:
        raise ValueError("--rate must be > 0")
    if args.step_x <= 0 or args.step_y <= 0 or args.step_z <= 0:
        raise ValueError("--step-x/--step-y/--step-z must be > 0")

    from decider.interfaces.sim_client import SimClient

    robot_id = resolve_robot_id(args.team, args.number)
    client = SimClient(ip=args.ip, port=str(args.port))

    vx, vy, vz = 0.0, 0.0, 0.0
    period = 1.0 / args.rate

    print(f"Connected to tcp://{args.ip}:{args.port}")
    print(f"Target robot: team={args.team}, number={args.number}, resolved id={robot_id}")
    print(f"Limits: x in [{-MAX_X}, {MAX_X}], y in [{-MAX_Y}, {MAX_Y}], z in [{-MAX_Z}, {MAX_Z}]")
    print_help()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        while True:
            key = read_key(period)
            if key:
                if key == "w":
                    vx = clamp(vx + args.step_x, -MAX_X, MAX_X)
                elif key == "s":
                    vx = clamp(vx - args.step_x, -MAX_X, MAX_X)
                elif key == "a":
                    vy = clamp(vy + args.step_y, -MAX_Y, MAX_Y)
                elif key == "d":
                    vy = clamp(vy - args.step_y, -MAX_Y, MAX_Y)
                elif key == "q":
                    vz = clamp(vz + args.step_z, -MAX_Z, MAX_Z)
                elif key == "e":
                    vz = clamp(vz - args.step_z, -MAX_Z, MAX_Z)
                elif key == " ":
                    vx, vy, vz = 0.0, 0.0, 0.0
                elif key == "h":
                    print_help()
                elif key == "x":
                    break

            resp = client.communicate([vx, vy, vz], robot_id=robot_id)
            sim_t = 0.0
            if isinstance(resp, dict):
                sim_t = float(resp.get("sim_timestamp", 0.0))
            print(
                f"\rteam={args.team} num={args.number} id={robot_id} "
                f"cmd=[{vx:+.2f}, {vy:+.2f}, {vz:+.2f}] sim_t={sim_t:.3f}",
                end="",
                flush=True,
            )
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        client.communicate([0.0, 0.0, 0.0], robot_id=robot_id)
        client.close()
        print("\nStopped.")


if __name__ == "__main__":
    main()
