# SPDX-FileCopyrightText: Copyright (c) MOS-Brain Contributors
# SPDX-License-Identifier: GPL-3.0-or-later

from __future__ import annotations

import argparse
import re
import sys
import types
from dataclasses import dataclass
from pathlib import Path

import numpy as np

K1_JOINTS_POLICY_ORDER = [
    "AAHead_yaw",
    "ALeft_Shoulder_Pitch",
    "ARight_Shoulder_Pitch",
    "Left_Hip_Pitch",
    "Right_Hip_Pitch",
    "Head_pitch",
    "Left_Shoulder_Roll",
    "Right_Shoulder_Roll",
    "Left_Hip_Roll",
    "Right_Hip_Roll",
    "Left_Elbow_Pitch",
    "Right_Elbow_Pitch",
    "Left_Hip_Yaw",
    "Right_Hip_Yaw",
    "Left_Elbow_Yaw",
    "Right_Elbow_Yaw",
    "Left_Knee_Pitch",
    "Right_Knee_Pitch",
    "Left_Ankle_Pitch",
    "Right_Ankle_Pitch",
    "Left_Ankle_Roll",
    "Right_Ankle_Roll",
]
OBS_JOINT_ORDER = K1_JOINTS_POLICY_ORDER

OBS_TERMS_ORDER = [
    "base_lin_vel",
    "base_ang_vel",
    "gravity_orientation",
    "cmd",
    "joint_pos",
    "joint_vel",
    "last_action",
]

OBS_SCALE = {
    "base_lin_vel": 1.0,
    "base_ang_vel": 0.2,
    "gravity_orientation": 1.0,
    "cmd": 1.0,
    "joint_pos": 1.0,
    "joint_vel": 0.05,
    "last_action": 1.0,
}

K1_ACTION_SCALE = {
    ".*Head.*": 0.375,
    ".*Shoulder_Pitch": 0.875,
    ".*Shoulder_Roll": 0.875,
    ".*Elbow_Pitch": 0.875,
    ".*Elbow_Yaw": 0.875,
    ".*Hip_Pitch": 0.09375,
    ".*Hip_Roll": 0.109375,
    ".*Hip_Yaw": 0.0625,
    ".*Knee_Pitch": 0.125,
    ".*Ankle_Pitch": 1.0 / 6.0,
    ".*Ankle_Roll": 1.0 / 6.0,
}

MOTOR_EFFORT_LIMIT = {
    ".*Head.*": 6.0,
    ".*Shoulder.*": 14.0,
    ".*Elbow.*": 14.0,
    ".*Hip_Pitch": 30.0,
    ".*Hip_Roll": 35.0,
    ".*Hip_Yaw": 20.0,
    ".*Knee_Pitch": 40.0,
    ".*Ankle_.*": 20.0,
}

MOTOR_STIFFNESS = {
    ".*Head.*": 4.0,
    ".*Shoulder.*": 4.0,
    ".*Elbow.*": 4.0,
    ".*Hip_Pitch": 80.0,
    ".*Hip_Roll": 80.0,
    ".*Hip_Yaw": 80.0,
    ".*Knee_Pitch": 80.0,
    ".*Ankle_.*": 30.0,
}

MOTOR_DAMPING = {
    ".*Head.*": 1.0,
    ".*Shoulder.*": 1.0,
    ".*Elbow.*": 1.0,
    ".*Hip_Pitch": 2.0,
    ".*Hip_Roll": 2.0,
    ".*Hip_Yaw": 2.0,
    ".*Knee_Pitch": 2.0,
    ".*Ankle_.*": 2.0,
}

PITCH_SCALE = 0.45
SIM_DT = 0.005
CONTROL_DECIMATION = 4
ACTION_CLIP = (-100.0, 100.0)
ACTION_SMOOTH_FILTER = False
DEFAULT_CMD = [0.0, 0.0, 0.0]
USE_BODY_VEL_OBS = True
RELOCATION_HOLD_SEC = 0.5
MAX_ROBOTS_PER_TEAM = 7
DEFAULT_POS = np.array([-3.5, 0.0, 0.57], dtype=np.float32)
SLOWDOWN_FACTOR = 1.0

FIXED_ROBOT_ID_TO_NAME = {
    **{i: f"robot_rp{i}" for i in range(MAX_ROBOTS_PER_TEAM)},
    **{MAX_ROBOTS_PER_TEAM + i: f"robot_bp{i}" for i in range(MAX_ROBOTS_PER_TEAM)},
}
FIXED_ROBOT_NAME_TO_ID = {name: rid for rid, name in FIXED_ROBOT_ID_TO_NAME.items()}


@dataclass
class RuntimeArgs:
    policy: Path
    robot_xml: Path
    soccer_world_xml: Path
    match_config: Path
    webview: bool
    zmq: bool
    webview_port: int
    web_fps: int
    web_width: int
    web_height: int
    port: int
    team_size: int
    max_red_robots: int
    max_blue_robots: int
    use_referee: bool
    policy_device: str


def _clamp_team_count(v: int) -> int:
    return max(0, min(MAX_ROBOTS_PER_TEAM, int(v)))


def parse_runtime_args(mujoco_dir: Path) -> RuntimeArgs:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", type=Path, default=mujoco_dir / "assets" / "policies" / "model_46000.pt")
    parser.add_argument(
        "--robot-xml",
        type=Path,
        default=mujoco_dir / "assets" / "robots" / "k1" / "K1_22dof.xml",
    )
    parser.add_argument(
        "--soccer-world-xml",
        type=Path,
        default=mujoco_dir / "assets" / "environments" / "soccer" / "world.xml",
    )
    parser.add_argument(
        "--match-config",
        type=Path,
        default=mujoco_dir / "assets" / "config" / "match_config.json",
    )
    parser.add_argument("--team-size", type=int, default=1, help="Robots per team (0-7). Red/Blue are equal.")
    parser.add_argument("--webview", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--zmq", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--webview-port", type=int, default=5811)
    parser.add_argument("--web-fps", type=int, default=20)
    parser.add_argument("--web-width", type=int, default=1280)
    parser.add_argument("--web-height", type=int, default=720)
    parser.add_argument("--port", type=int, default=5555, help="ZeroMQ REP port.")
    parser.add_argument("--use-referee", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--policy-device",
        type=str,
        choices=["cpu", "gpu"],
        default="gpu",
        help="Policy inference device. If set to gpu but CUDA is unavailable, falls back to CPU.",
    )
    ns = parser.parse_args()
    team_size = _clamp_team_count(ns.team_size)
    return RuntimeArgs(
        policy=ns.policy,
        robot_xml=ns.robot_xml,
        soccer_world_xml=ns.soccer_world_xml,
        match_config=ns.match_config,
        webview=ns.webview,
        zmq=ns.zmq,
        webview_port=ns.webview_port,
        web_fps=ns.web_fps,
        web_width=ns.web_width,
        web_height=ns.web_height,
        port=ns.port,
        team_size=team_size,
        max_red_robots=team_size,
        max_blue_robots=team_size,
        use_referee=ns.use_referee,
        policy_device=ns.policy_device,
    )


def build_action_scale_array(policy_joint_names: list[str], scale_cfg: dict[str, float]) -> np.ndarray:
    scales = np.zeros(len(policy_joint_names), dtype=np.float32)
    for i, joint_name in enumerate(policy_joint_names):
        for pattern, val in scale_cfg.items():
            if re.match(pattern, joint_name):
                scales[i] = float(val)
                break
        if scales[i] == 0.0:
            raise ValueError(f"No action scale matched for joint: {joint_name}")
    return scales


def parse_param_for_joint_names(joint_names: list[str], param: float | dict[str, float]) -> np.ndarray:
    out = np.zeros(len(joint_names), dtype=np.float32)
    if isinstance(param, (float, int)):
        out.fill(float(param))
        return out
    if not isinstance(param, dict):
        raise ValueError(f"Unsupported parameter type: {type(param)}")
    for i, name in enumerate(joint_names):
        matched = False
        for pattern, value in param.items():
            if re.match(pattern, name):
                out[i] = float(value)
                matched = True
                break
        if not matched:
            out[i] = 1e-7
    return out


def _ensure_trackerlab_stub() -> None:
    if "trackerLab.managers.motion_manager.motion_manager_cfg" in sys.modules:
        return
    trackerlab_pkg = sys.modules.setdefault("trackerLab", types.ModuleType("trackerLab"))
    managers_pkg = sys.modules.setdefault("trackerLab.managers", types.ModuleType("trackerLab.managers"))
    motion_pkg = sys.modules.setdefault("trackerLab.managers.motion_manager", types.ModuleType("trackerLab.managers.motion_manager"))
    motion_cfg_mod = types.ModuleType("trackerLab.managers.motion_manager.motion_manager_cfg")

    class MotionManagerCfg:
        pass

    motion_cfg_mod.MotionManagerCfg = MotionManagerCfg
    sys.modules["trackerLab.managers.motion_manager.motion_manager_cfg"] = motion_cfg_mod
    trackerlab_pkg.managers = managers_pkg
    managers_pkg.motion_manager = motion_pkg


def build_sim2sim_cfg(scene_xml: Path, policy_path: Path):
    _ensure_trackerlab_stub()
    from sim2simlib.model.actuator_motor import PIDMotor
    from sim2simlib.model.config import Actions_Config, Motor_Config, Observations_Config, Sim2Sim_Config

    return Sim2Sim_Config(
        robot_name="k1",
        simulation_dt=SIM_DT,
        control_decimation=CONTROL_DECIMATION,
        slowdown_factor=SLOWDOWN_FACTOR,
        xml_path=str(scene_xml),
        policy_path=str(policy_path),
        policy_joint_names=K1_JOINTS_POLICY_ORDER,
        default_pos=DEFAULT_POS.copy(),
        # Force both arms down at startup.
        default_angles={
            r".*": 0.0,
            r"^Left_Shoulder_Roll$": -1.3,
            r"^Right_Shoulder_Roll$": 1.3,
        },
        observation_cfg=Observations_Config(
            base_observations_terms=OBS_TERMS_ORDER,
            scale=OBS_SCALE,
            using_base_obs_history=False,
            base_obs_flatten=True,
            base_obs_his_length=1,
        ),
        action_cfg=Actions_Config(
            scale=build_action_scale_array(K1_JOINTS_POLICY_ORDER, K1_ACTION_SCALE),
            action_clip=ACTION_CLIP,
            smooth_filter=ACTION_SMOOTH_FILTER,
        ),
        motor_cfg=Motor_Config(
            motor_type=PIDMotor,
            effort_limit=MOTOR_EFFORT_LIMIT,
            stiffness=MOTOR_STIFFNESS,
            damping=MOTOR_DAMPING,
            saturation_effort=MOTOR_EFFORT_LIMIT,
            velocity_limit=40.0,
            friction=0.0,
        ),
        cmd=DEFAULT_CMD.copy(),
    )
