# SPDX-FileCopyrightText: Copyright (c) MOS-Brain Contributors
# SPDX-License-Identifier: GPL-3.0-or-later

from __future__ import annotations

import json
import re
import sys
import tempfile
import time
import types
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import torch
import torch.nn as nn
import zmq

from .runtime_config import (
    ACTION_CLIP,
    ACTION_SMOOTH_FILTER,
    DEFAULT_CMD,
    FIXED_ROBOT_ID_TO_NAME,
    FIXED_ROBOT_NAME_TO_ID,
    MAX_ROBOTS_PER_TEAM,
    PI_PLUS_KD_POLICY_ORDER,
    PI_PLUS_KP_POLICY_ORDER,
    PI_PLUS_ROBOT_TYPE,
    PITCH_SCALE,
    RobotRuntimeConfig,
    RuntimeArgs,
    build_action_scale_array,
    parse_param_for_joint_names,
)
from .soccer_referee import MujocoSoccerReferee
from .webview_server import MujocoLabWebView


FIELD_PRESETS = {
    "S": (9.0, 6.0),
    "M": (14.0, 9.0),
    "L": (22.0, 14.0),
}

DRAG_RESET_PROTECT_SEC = 0.5
DRAG_CMD_ZERO_POLICY_FRAMES = 5


def _load_checkpoint_compat(path: Path, map_location: torch.device):
    try:
        return torch.load(path, map_location=map_location, weights_only=False)
    except ModuleNotFoundError as e:
        # pi_plus checkpoints may include rsl_rl.utils.utils.Normalizer in pickle payload.
        if "rsl_rl" not in str(e):
            raise
        rsl_rl_mod = types.ModuleType("rsl_rl")
        utils_pkg = types.ModuleType("rsl_rl.utils")
        utils_mod = types.ModuleType("rsl_rl.utils.utils")

        class Normalizer:
            pass

        Normalizer.__module__ = "rsl_rl.utils.utils"
        utils_mod.Normalizer = Normalizer
        rsl_rl_mod.utils = utils_pkg
        utils_pkg.utils = utils_mod
        sys.modules.setdefault("rsl_rl", rsl_rl_mod)
        sys.modules.setdefault("rsl_rl.utils", utils_pkg)
        sys.modules.setdefault("rsl_rl.utils.utils", utils_mod)
        return torch.load(path, map_location=map_location, weights_only=False)


def _load_field_size_from_match_config(match_config_path: Path | None) -> tuple[float, float] | None:
    if match_config_path is None or not match_config_path.exists():
        return None
    try:
        data = json.loads(match_config_path.read_text(encoding="utf-8"))
        field_cfg = data.get("field", {})
        preset = str(field_cfg.get("preset", "M")).upper()
        if preset in FIELD_PRESETS:
            return FIELD_PRESETS[preset]
        length = field_cfg.get("length")
        width = field_cfg.get("width")
        if length is not None and width is not None:
            return float(length), float(width)
    except Exception:
        pass
    return None


def _load_goal_config_from_match_config(match_config_path: Path | None) -> dict[str, float]:
    cfg = {
        "depth": 0.6,
        "width": 2.6,
        "height": 1.8,
        "post_radius": 0.05,
    }
    if match_config_path is None or not match_config_path.exists():
        return cfg
    try:
        data = json.loads(match_config_path.read_text(encoding="utf-8"))
        field_cfg = data.get("field", {}) if isinstance(data, dict) else {}
        goal_cfg = field_cfg.get("goal", data.get("goal", {}))
        if not isinstance(goal_cfg, dict):
            return cfg
        if "depth" in goal_cfg:
            cfg["depth"] = float(goal_cfg["depth"])
        if "width" in goal_cfg:
            cfg["width"] = float(goal_cfg["width"])
        if "height" in goal_cfg:
            cfg["height"] = float(goal_cfg["height"])
        if "post_radius" in goal_cfg:
            cfg["post_radius"] = float(goal_cfg["post_radius"])
    except Exception:
        pass
    return cfg


def _load_outer_floor_config_from_match_config(match_config_path: Path | None) -> dict[str, object]:
    cfg: dict[str, object] = {
        "enabled": True,
        "margin_ratio": 0.05,
        "min_margin": 1.0,
        "color": [0.2, 0.5, 0.2, 1.0],
        "collision": False,
        "edge_walls_enabled": True,
        "edge_wall_height": 0.8,
        "edge_wall_thickness": 0.04,
        "edge_wall_color": [0.8, 0.9, 1.0, 0.12],
        "edge_wall_collision": True,
    }
    if match_config_path is None or not match_config_path.exists():
        return cfg
    try:
        data = json.loads(match_config_path.read_text(encoding="utf-8"))
        field_cfg = data.get("field", {}) if isinstance(data, dict) else {}
        outer = field_cfg.get("outer_floor", {})
        if not isinstance(outer, dict):
            return cfg
        if "enabled" in outer:
            cfg["enabled"] = bool(outer["enabled"])
        if "margin_ratio" in outer:
            cfg["margin_ratio"] = float(outer["margin_ratio"])
        if "min_margin" in outer:
            cfg["min_margin"] = float(outer["min_margin"])
        if "collision" in outer:
            cfg["collision"] = bool(outer["collision"])
        if "color" in outer and isinstance(outer["color"], (list, tuple)) and len(outer["color"]) == 4:
            cfg["color"] = [float(x) for x in outer["color"]]
        if "edge_walls_enabled" in outer:
            cfg["edge_walls_enabled"] = bool(outer["edge_walls_enabled"])
        if "edge_wall_height" in outer:
            cfg["edge_wall_height"] = float(outer["edge_wall_height"])
        if "edge_wall_thickness" in outer:
            cfg["edge_wall_thickness"] = float(outer["edge_wall_thickness"])
        if "edge_wall_collision" in outer:
            cfg["edge_wall_collision"] = bool(outer["edge_wall_collision"])
        if "edge_wall_color" in outer and isinstance(outer["edge_wall_color"], (list, tuple)) and len(outer["edge_wall_color"]) == 4:
            cfg["edge_wall_color"] = [float(x) for x in outer["edge_wall_color"]]
    except Exception:
        pass
    return cfg


def _write_temp_xml(xml_text: str) -> Path:
    fd = tempfile.NamedTemporaryFile(prefix="multi_k1_scene_", suffix=".xml", delete=False)
    fd.write(xml_text.encode("utf-8"))
    fd.flush()
    fd.close()
    return Path(fd.name)


def _ensure_offscreen_buffer(root: ET.Element, offwidth: int = 1920, offheight: int = 1080):
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    global_tag = visual.find("global")
    if global_tag is None:
        global_tag = ET.SubElement(visual, "global")
    cur_w = int(global_tag.get("offwidth", "0") or "0")
    cur_h = int(global_tag.get("offheight", "0") or "0")
    global_tag.set("offwidth", str(max(cur_w, offwidth)))
    global_tag.set("offheight", str(max(cur_h, offheight)))


def _remove_all_plane_geoms(root: ET.Element):
    # Remove every plane geom defined in robot XML so only soccer world pitch remains.
    for worldbody in list(root.findall("worldbody")):
        for geom in list(worldbody.iter("geom")):
            if geom.get("type") != "plane":
                continue
            parent = next((p for p in worldbody.iter() if geom in list(p)), None)
            if parent is not None:
                parent.remove(geom)


def _find_template_robot_body(worldbody: ET.Element, base_joint_name: str) -> ET.Element:
    for body in list(worldbody.findall("body")):
        if body.find(f"joint[@name='{base_joint_name}']") is not None or body.find(
            f"freejoint[@name='{base_joint_name}']"
        ) is not None:
            return body
    raise RuntimeError(f"Cannot find template robot body with base joint '{base_joint_name}' in robot XML")


def _prefix_body_tree_names(body: ET.Element, robot_name: str):
    for elem in body.iter():
        name = elem.get("name")
        if not name:
            continue
        if elem.tag == "body":
            if elem is body:
                elem.set("name", robot_name)
            else:
                elem.set("name", f"{robot_name}__{name}")
        elif elem.tag in ("joint", "freejoint", "site", "geom", "camera", "light"):
            elem.set("name", f"{robot_name}__{name}")


def _spawn_xy_theta(team: str, idx: int, count: int, field_size: tuple[float, float] | None) -> tuple[float, float, float]:
    field_len = float(field_size[0]) if field_size is not None else 14.0
    y_spacing = 1.0
    start_y = -((count - 1) * y_spacing) * 0.5
    y = start_y + idx * y_spacing
    if team == "red":
        return (-field_len * 0.25, y, 0.0)
    return (field_len * 0.25, y, np.pi)


def _quat_from_yaw(theta: float) -> np.ndarray:
    half = 0.5 * float(theta)
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)], dtype=np.float32)


def _add_procedural_goals(
    worldbody: ET.Element,
    field_length: float,
    goal_depth: float = 0.6,
    goal_width: float = 2.6,
    goal_height: float = 1.8,
    post_radius: float = 0.05,
):
    goal_half_y = 0.5 * float(goal_width)
    field_half_x = 0.5 * float(field_length)

    post_rgba = "0.8 0.8 0.8 1"
    net_rgba = "1 1 1 0.2"
    y_quat = "0 0 0.7071068 0.7071068"

    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        goal_name = f"goal-{side}"
        goal_body = ET.SubElement(worldbody, "body", name=goal_name, pos=f"{x_sign * field_half_x:g} 0 0")
        depth = x_sign * float(goal_depth)
        x_half_depth = 0.5 * abs(depth)

        # Vertical posts
        ET.SubElement(
            goal_body,
            "geom",
            name=f"{goal_name}-front-left-post",
            type="cylinder",
            pos=f"0 {-goal_half_y:g} {0.5 * goal_height:g}",
            size=f"{post_radius:g} {0.5 * goal_height:g}",
            rgba=post_rgba,
        )
        ET.SubElement(
            goal_body,
            "geom",
            name=f"{goal_name}-front-right-post",
            type="cylinder",
            pos=f"0 {goal_half_y:g} {0.5 * goal_height:g}",
            size=f"{post_radius:g} {0.5 * goal_height:g}",
            rgba=post_rgba,
        )
        ET.SubElement(
            goal_body,
            "geom",
            name=f"{goal_name}-back-left-post",
            type="cylinder",
            pos=f"{depth:g} {-goal_half_y:g} {0.5 * goal_height:g}",
            size=f"{post_radius:g} {0.5 * goal_height:g}",
            rgba=post_rgba,
        )
        ET.SubElement(
            goal_body,
            "geom",
            name=f"{goal_name}-back-right-post",
            type="cylinder",
            pos=f"{depth:g} {goal_half_y:g} {0.5 * goal_height:g}",
            size=f"{post_radius:g} {0.5 * goal_height:g}",
            rgba=post_rgba,
        )

        # Crossbars
        ET.SubElement(
            goal_body,
            "geom",
            name=f"{goal_name}-front-crossbar",
            type="cylinder",
            pos=f"0 0 {goal_height:g}",
            size=f"{post_radius:g} {goal_half_y:g}",
            quat=y_quat,
            rgba=post_rgba,
        )
        ET.SubElement(
            goal_body,
            "geom",
            name=f"{goal_name}-back-crossbar",
            type="cylinder",
            pos=f"{depth:g} 0 {goal_height:g}",
            size=f"{post_radius:g} {goal_half_y:g}",
            quat=y_quat,
            rgba=post_rgba,
        )
        ET.SubElement(
            goal_body,
            "geom",
            name=f"{goal_name}-left-side-crossbar",
            type="cylinder",
            pos=f"{0.5 * depth:g} {-goal_half_y:g} {goal_height:g}",
            size=f"{post_radius:g} {x_half_depth:g}",
            quat=f"0 {0.7071068 * x_sign:g} 0 0.7071068",
            rgba=post_rgba,
        )
        ET.SubElement(
            goal_body,
            "geom",
            name=f"{goal_name}-right-side-crossbar",
            type="cylinder",
            pos=f"{0.5 * depth:g} {goal_half_y:g} {goal_height:g}",
            size=f"{post_radius:g} {x_half_depth:g}",
            quat=f"0 {-0.7071068 * x_sign:g} 0 0.7071068",
            rgba=post_rgba,
        )

        # Light-weight visual nets (non-colliding)
        ET.SubElement(
            goal_body,
            "geom",
            name=f"{goal_name}-top-net",
            type="box",
            pos=f"{0.5 * depth:g} 0 {goal_height:g}",
            size=f"{x_half_depth:g} {goal_half_y:g} {post_radius:g}",
            rgba=net_rgba,
            contype="0",
            conaffinity="0",
        )
        ET.SubElement(
            goal_body,
            "geom",
            name=f"{goal_name}-back-net",
            type="box",
            pos=f"{depth:g} 0 {0.5 * goal_height:g}",
            size=f"{post_radius:g} {goal_half_y:g} {0.5 * goal_height:g}",
            rgba=net_rgba,
            contype="0",
            conaffinity="0",
        )


def _add_outer_floor_planes(
    worldbody: ET.Element,
    field_length: float,
    field_width: float,
    cfg: dict[str, object] | None = None,
):
    c = cfg if isinstance(cfg, dict) else {}
    if not bool(c.get("enabled", True)):
        return
    ratio = float(c.get("margin_ratio", 0.05))
    min_margin = float(c.get("min_margin", 1.0))
    margin_x = max(min_margin, 0.5 * field_length * ratio)
    margin_y = max(min_margin, 0.5 * field_width * ratio)
    field_half_x = 0.5 * float(field_length)
    field_half_y = 0.5 * float(field_width)
    rgba = c.get("color", [0.2, 0.5, 0.2, 1.0])
    if not isinstance(rgba, (list, tuple)) or len(rgba) != 4:
        rgba = [0.2, 0.5, 0.2, 1.0]
    rgba_str = " ".join(f"{float(v):g}" for v in rgba)
    coll = bool(c.get("collision", False))
    contype = "1" if coll else "0"
    conaffinity = "1" if coll else "0"

    # Left / right
    ET.SubElement(
        worldbody,
        "geom",
        name="left-floor",
        type="plane",
        pos=f"{-field_half_x - margin_x:g} 0 0",
        size=f"{margin_x:g} {field_half_y:g} 1",
        rgba=rgba_str,
        contype=contype,
        conaffinity=conaffinity,
    )
    ET.SubElement(
        worldbody,
        "geom",
        name="right-floor",
        type="plane",
        pos=f"{field_half_x + margin_x:g} 0 0",
        size=f"{margin_x:g} {field_half_y:g} 1",
        rgba=rgba_str,
        contype=contype,
        conaffinity=conaffinity,
    )
    # Top / bottom
    ET.SubElement(
        worldbody,
        "geom",
        name="top-floor",
        type="plane",
        pos=f"0 {field_half_y + margin_y:g} 0",
        size=f"{field_half_x + 2.0 * margin_x:g} {margin_y:g} 1",
        rgba=rgba_str,
        contype=contype,
        conaffinity=conaffinity,
    )
    ET.SubElement(
        worldbody,
        "geom",
        name="bottom-floor",
        type="plane",
        pos=f"0 {-field_half_y - margin_y:g} 0",
        size=f"{field_half_x + 2.0 * margin_x:g} {margin_y:g} 1",
        rgba=rgba_str,
        contype=contype,
        conaffinity=conaffinity,
    )

    # Add transparent boundary walls on outer-floor edges to keep robots inside controllable area.
    if bool(c.get("edge_walls_enabled", True)):
        wall_h = max(0.05, float(c.get("edge_wall_height", 0.8)))
        wall_t = max(0.01, float(c.get("edge_wall_thickness", 0.04)))
        wall_rgba = c.get("edge_wall_color", [0.8, 0.9, 1.0, 0.12])
        if not isinstance(wall_rgba, (list, tuple)) or len(wall_rgba) != 4:
            wall_rgba = [0.8, 0.9, 1.0, 0.12]
        wall_rgba_str = " ".join(f"{float(v):g}" for v in wall_rgba)
        wall_coll = bool(c.get("edge_wall_collision", True))
        wall_contype = "1" if wall_coll else "0"
        wall_conaffinity = "1" if wall_coll else "0"

        outer_half_x = field_half_x + 2.0 * margin_x
        outer_half_y = field_half_y + 2.0 * margin_y
        half_h = 0.5 * wall_h
        half_t = 0.5 * wall_t

        # Left/right walls (normal along X)
        ET.SubElement(
            worldbody,
            "geom",
            name="outer-wall-left",
            type="box",
            pos=f"{-outer_half_x - half_t:g} 0 {half_h:g}",
            size=f"{half_t:g} {outer_half_y:g} {half_h:g}",
            rgba=wall_rgba_str,
            contype=wall_contype,
            conaffinity=wall_conaffinity,
        )
        ET.SubElement(
            worldbody,
            "geom",
            name="outer-wall-right",
            type="box",
            pos=f"{outer_half_x + half_t:g} 0 {half_h:g}",
            size=f"{half_t:g} {outer_half_y:g} {half_h:g}",
            rgba=wall_rgba_str,
            contype=wall_contype,
            conaffinity=wall_conaffinity,
        )

        # Top/bottom walls (normal along Y)
        ET.SubElement(
            worldbody,
            "geom",
            name="outer-wall-top",
            type="box",
            pos=f"0 {outer_half_y + half_t:g} {half_h:g}",
            size=f"{outer_half_x:g} {half_t:g} {half_h:g}",
            rgba=wall_rgba_str,
            contype=wall_contype,
            conaffinity=wall_conaffinity,
        )
        ET.SubElement(
            worldbody,
            "geom",
            name="outer-wall-bottom",
            type="box",
            pos=f"0 {-outer_half_y - half_t:g} {half_h:g}",
            size=f"{outer_half_x:g} {half_t:g} {half_h:g}",
            rgba=wall_rgba_str,
            contype=wall_contype,
            conaffinity=wall_conaffinity,
        )


def _build_multi_robot_soccer_scene_xml(
    robot_xml: Path,
    soccer_world_xml: Path,
    max_red_robots: int,
    max_blue_robots: int,
    base_joint_name: str,
    pitch_scale: float = PITCH_SCALE,
    target_field_size: tuple[float, float] | None = None,
    goal_cfg: dict[str, float] | None = None,
    outer_floor_cfg: dict[str, object] | None = None,
) -> tuple[Path, list[int]]:
    meshdir = robot_xml.parent / "meshes"
    robot_root = ET.fromstring(robot_xml.read_text(encoding="utf-8"))
    world_root = ET.parse(soccer_world_xml).getroot()

    robot_compiler = robot_root.find("compiler")
    if robot_compiler is not None:
        robot_compiler.set("meshdir", meshdir.as_posix())

    _ensure_offscreen_buffer(robot_root)
    _remove_all_plane_geoms(robot_root)

    worldbody = robot_root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Robot XML missing worldbody")

    template_body = _find_template_robot_body(worldbody, base_joint_name=base_joint_name)
    template_actuator = robot_root.find("actuator")
    if template_actuator is None:
        raise RuntimeError("Robot XML missing actuator section")
    template_actuators = list(template_actuator)

    worldbody.remove(template_body)
    for child in list(template_actuator):
        template_actuator.remove(child)

    sensor = robot_root.find("sensor")
    if sensor is not None:
        robot_root.remove(sensor)

    active_robot_ids: list[int] = []
    template_pos_vals = [float(v) for v in (template_body.get("pos", "0 0 0").split())]
    template_spawn_z = template_pos_vals[2] if len(template_pos_vals) >= 3 else 0.0

    def add_team(team: str, count: int):
        base_id = 0 if team == "red" else MAX_ROBOTS_PER_TEAM
        for i in range(count):
            rid = base_id + i
            robot_name = FIXED_ROBOT_ID_TO_NAME[rid]
            body_copy = deepcopy(template_body)
            _prefix_body_tree_names(body_copy, robot_name)
            x, y, theta = _spawn_xy_theta(team, i, count, target_field_size)
            body_copy.set("pos", f"{x:.6f} {y:.6f} {template_spawn_z:.6f}")
            body_copy.set("quat", " ".join(f"{v:.9g}" for v in _quat_from_yaw(theta)))
            worldbody.append(body_copy)

            for act in template_actuators:
                act_copy = deepcopy(act)
                if act_copy.get("name"):
                    act_copy.set("name", f"{robot_name}__{act_copy.get('name')}")
                if act_copy.get("joint"):
                    act_copy.set("joint", f"{robot_name}__{act_copy.get('joint')}")
                template_actuator.append(act_copy)

            active_robot_ids.append(rid)

    add_team("red", max_red_robots)
    add_team("blue", max_blue_robots)

    world_compiler = world_root.find("compiler")
    world_asset_dir = soccer_world_xml.parent
    if world_compiler is not None and world_compiler.get("assetdir"):
        world_asset_dir = soccer_world_xml.parent / world_compiler.get("assetdir")

    robot_asset = robot_root.find("asset")
    if robot_asset is None:
        robot_asset = ET.SubElement(robot_root, "asset")
    world_asset = world_root.find("asset")
    if world_asset is not None:
        for child in list(world_asset):
            copied = deepcopy(child)
            for attr in ("file", "fileup", "filedown", "filefront", "fileback", "fileleft", "fileright"):
                v = copied.get(attr)
                if v and not Path(v).is_absolute():
                    copied.set(attr, (world_asset_dir / v).as_posix())
            robot_asset.append(copied)

    world_worldbody = world_root.find("worldbody")
    out_field_len = float(target_field_size[0]) if target_field_size is not None else 14.0
    out_field_wid = float(target_field_size[1]) if target_field_size is not None else 9.0
    if world_worldbody is not None:
        for child in list(world_worldbody):
            copied = deepcopy(child)
            if copied.tag == "geom" and copied.get("name") == "pitch":
                size_str = copied.get("size")
                if size_str:
                    vals = [float(x) for x in size_str.split()]
                    if len(vals) >= 2:
                        if target_field_size is not None:
                            vals[0] = float(target_field_size[0]) * 0.5
                            vals[1] = float(target_field_size[1]) * 0.5
                        else:
                            vals[0] *= pitch_scale
                            vals[1] *= pitch_scale
                        copied.set("size", " ".join(f"{v:g}" for v in vals))
                        out_field_len = float(vals[0]) * 2.0
                        out_field_wid = float(vals[1]) * 2.0
                # Increase rolling friction on pitch so kicked ball decelerates sooner.
                copied.set("friction", "1.0 0.2 0.03")
                copied.set("solref", "0.001 1")
                copied.set("solimp", "0.9 0.95 0.001")
            worldbody.append(copied)

    _add_outer_floor_planes(worldbody, field_length=out_field_len, field_width=out_field_wid, cfg=outer_floor_cfg)

    g = goal_cfg if isinstance(goal_cfg, dict) else {}
    _add_procedural_goals(
        worldbody,
        field_length=out_field_len,
        goal_depth=float(g.get("depth", 0.6)),
        goal_width=float(g.get("width", 2.6)),
        goal_height=float(g.get("height", 1.8)),
        post_radius=float(g.get("post_radius", 0.05)),
    )

    xml_path = _write_temp_xml(ET.tostring(robot_root, encoding="unicode"))
    return xml_path, active_robot_ids


class MLPActor(nn.Module):
    def __init__(self, layer_dims: list[int]):
        super().__init__()
        if len(layer_dims) < 2:
            raise ValueError("Actor layer_dims must contain at least input and output sizes")
        layers: list[nn.Module] = []
        for i in range(len(layer_dims) - 1):
            layers.append(nn.Linear(layer_dims[i], layer_dims[i + 1]))
            if i < len(layer_dims) - 2:
                layers.append(nn.ELU())
        self.actor = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.actor(x)


def _quat_to_rot_world_from_body(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_wxyz
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


@dataclass
class RobotSpec:
    rid: int
    name: str
    team: str
    qpos_idx: np.ndarray
    qvel_idx: np.ndarray
    act_idx: np.ndarray
    act_qpos_idx: np.ndarray
    act_qvel_idx: np.ndarray
    base_qpos_adr: int
    base_qvel_adr: int
    init_joint_pos: np.ndarray
    init_angles: np.ndarray
    filtered_dof_target: np.ndarray
    target_joint_pos: np.ndarray
    last_action: np.ndarray
    action_scale: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    effort: np.ndarray
    obs_step_dim: int
    obs_history: np.ndarray


class MultiRobotMujocoSim:
    def __init__(self, args: RuntimeArgs):
        self.args = args
        self.robot_cfg: RobotRuntimeConfig = args.robot_cfg
        self.max_red_robots = args.max_red_robots
        self.max_blue_robots = args.max_blue_robots
        self.active_robot_ids = self._active_ids_from_limits(self.max_red_robots, self.max_blue_robots)

        field_size = _load_field_size_from_match_config(args.match_config)
        self._field_length = float(field_size[0]) if field_size is not None else 14.0
        self._field_width = float(field_size[1]) if field_size is not None else 9.0
        goal_cfg = _load_goal_config_from_match_config(args.match_config)
        self._goal_width = float(goal_cfg.get("width", 2.6))
        self._goal_height = float(goal_cfg.get("height", 1.8))
        outer_floor_cfg = _load_outer_floor_config_from_match_config(args.match_config)
        scene_xml, _ = _build_multi_robot_soccer_scene_xml(
            args.robot_xml,
            args.soccer_world_xml,
            max_red_robots=self.max_red_robots,
            max_blue_robots=self.max_blue_robots,
            base_joint_name=self.robot_cfg.base_joint_name,
            pitch_scale=PITCH_SCALE,
            target_field_size=field_size,
            goal_cfg=goal_cfg,
            outer_floor_cfg=outer_floor_cfg,
        )

        self.model = mujoco.MjModel.from_xml_path(str(scene_xml))
        self.data = mujoco.MjData(self.model)
        self.sim_dt = float(self.robot_cfg.sim_dt)
        self.model.opt.timestep = self.sim_dt
        if self.robot_cfg.robot_type == PI_PLUS_ROBOT_TYPE:
            # Match sim2sim_pi_plus.py: only timestep is explicitly configured.
            # Keep integrator/noslip from model XML defaults.
            pass
        else:
            self.model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4
            self.model.opt.noslip_iterations = 100
        self.control_decimation = int(self.robot_cfg.control_decimation)

        self.policy_device = self._resolve_policy_device(args.policy_device)
        print(f"[MultiRobotMujocoSim] policy device: {self.policy_device}")
        self.policy = self._load_policy(args.policy)

        self.robot_specs: dict[int, RobotSpec] = self._build_robot_specs()
        if self.robot_specs:
            sample_spec = next(iter(self.robot_specs.values()))
            expected_obs_dim = len(sample_spec.obs_history)
            expected_act_dim = len(sample_spec.last_action)
            if expected_obs_dim != self._policy_obs_dim:
                raise RuntimeError(
                    f"Policy obs dim mismatch: policy={self._policy_obs_dim} robot={expected_obs_dim} type={self.robot_cfg.robot_type}"
                )
            if expected_act_dim != self._policy_action_dim:
                raise RuntimeError(
                    f"Policy action dim mismatch: policy={self._policy_action_dim} robot={expected_act_dim} type={self.robot_cfg.robot_type}"
                )
        self.command_buffer: dict[int, np.ndarray] = {
            rid: np.array(DEFAULT_CMD, dtype=np.float32) for rid in FIXED_ROBOT_ID_TO_NAME
        }
        self.command_ts: dict[int, float] = {rid: float("-inf") for rid in FIXED_ROBOT_ID_TO_NAME}
        self.command_received: dict[int, bool] = {rid: False for rid in FIXED_ROBOT_ID_TO_NAME}
        for rid in self.robot_specs:
            self.command_received[rid] = True
        self.last_msg_info = {"timestamp": 0.0, "id": -1, "source": "unknown"}
        self._policy_step_count = 0
        self._policy_print_step = 0
        self._printed_target_policy_io = False

        self._startup_qpos = self.data.qpos.copy()
        self._startup_qvel = self.data.qvel.copy()
        self._startup_ctrl = self.data.ctrl.copy()
        self._startup_act = self.data.act.copy() if self.data.act.size > 0 else np.array([], dtype=np.float32)
        self._saved_spawn_points: dict[str, list[float]] = {}
        self._robot_protect_until: dict[int, float] = {}
        self._robot_protect_pose: dict[int, tuple[float, float, float]] = {}
        self._robot_cmd_zero_frames_left: dict[int, int] = {}
        self._ball_last_touch_rid: int | None = None

        self.use_referee = bool(args.use_referee)
        self.referee: MujocoSoccerReferee | None = None
        if self.use_referee:
            self.referee = MujocoSoccerReferee(
                field_length=self._field_length,
                field_width=self._field_width,
                goal_width=self._goal_width,
                goal_height=self._goal_height,
                red_count=self.max_red_robots,
                blue_count=self.max_blue_robots,
            )
            print("[MultiRobotMujocoSim] referee: enabled")
        else:
            print("[MultiRobotMujocoSim] referee: disabled")

        self._web_camera = None

    @staticmethod
    def _active_ids_from_limits(max_red: int, max_blue: int) -> list[int]:
        ids = []
        ids.extend(range(0, max_red))
        ids.extend(range(MAX_ROBOTS_PER_TEAM, MAX_ROBOTS_PER_TEAM + max_blue))
        return ids

    @staticmethod
    def _resolve_policy_device(requested: str) -> torch.device:
        req = str(requested).strip().lower()
        if req == "cpu":
            return torch.device("cpu")
        if req == "gpu":
            if torch.cuda.is_available():
                return torch.device("cuda")
            print("[MultiRobotMujocoSim] policy device requested=gpu but CUDA is unavailable, fallback to cpu")
            return torch.device("cpu")
        raise ValueError(f"Unsupported policy device: {requested}")

    def _load_policy(self, policy_path: Path):
        ckpt = _load_checkpoint_compat(policy_path, map_location=self.policy_device)
        state_dict = ckpt["model_state_dict"] if isinstance(ckpt, dict) and "model_state_dict" in ckpt else ckpt
        if not isinstance(state_dict, dict):
            raise RuntimeError(f"Unsupported policy checkpoint format: {type(state_dict)}")
        actor_state = {k: v for k, v in state_dict.items() if k.startswith("actor.")}
        if not actor_state:
            raise RuntimeError("Checkpoint does not contain actor.* weights")

        actor_layer_dims: list[int] = []
        actor_weight_keys = sorted(
            (k for k in actor_state if re.match(r"^actor\.\d+\.weight$", k)),
            key=lambda s: int(s.split(".")[1]),
        )
        for i, wk in enumerate(actor_weight_keys):
            w = actor_state[wk]
            out_dim, in_dim = int(w.shape[0]), int(w.shape[1])
            if i == 0:
                actor_layer_dims.append(in_dim)
            actor_layer_dims.append(out_dim)

        self._policy_obs_dim = int(actor_layer_dims[0])
        self._policy_action_dim = int(actor_layer_dims[-1])
        policy = MLPActor(layer_dims=actor_layer_dims).to(self.policy_device)
        policy.load_state_dict(actor_state, strict=True)
        policy.eval()
        return policy

    def _build_robot_specs(self) -> dict[int, RobotSpec]:
        specs: dict[int, RobotSpec] = {}
        joint_names = self.robot_cfg.policy_joint_names
        action_scale = build_action_scale_array(joint_names, self.robot_cfg.action_scale_cfg)
        obs_step_dim = (9 if self.robot_cfg.include_base_lin_vel_obs else 6) + 3 + 3 * len(joint_names)
        obs_history_len = max(1, int(self.robot_cfg.obs_history_length))
        obs_dim = obs_step_dim * obs_history_len

        for rid in self.active_robot_ids:
            name = FIXED_ROBOT_ID_TO_NAME[rid]
            team = "red" if rid < MAX_ROBOTS_PER_TEAM else "blue"
            pref_joints = [f"{name}__{j}" for j in joint_names]
            qpos_idx = []
            qvel_idx = []
            for jn in pref_joints:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                if jid < 0:
                    raise RuntimeError(f"Missing policy joint: {jn}")
                qpos_idx.append(int(self.model.jnt_qposadr[jid]))
                qvel_idx.append(int(self.model.jnt_dofadr[jid]))

            act_idx: list[int] = []
            for jn in pref_joints:
                aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}__{jn.split('__', 1)[1]}")
                if aid < 0:
                    raise RuntimeError(f"Missing actuator for policy joint: {jn}")
                act_idx.append(int(aid))

            base_jid = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                f"{name}__{self.robot_cfg.base_joint_name}",
            )
            if base_jid < 0:
                raise RuntimeError(f"Missing base freejoint for {name}")
            base_qpos_adr = int(self.model.jnt_qposadr[base_jid])
            base_qvel_adr = int(self.model.jnt_dofadr[base_jid])

            init_joint_pos = self.data.qpos[np.asarray(qpos_idx, dtype=np.int32)].astype(np.float32).copy()
            # Enforce requested startup/reset joint pose for every robot.
            for jname, val in self.robot_cfg.reset_joint_pos.items():
                try:
                    local_idx = joint_names.index(jname)
                except ValueError:
                    continue
                init_joint_pos[local_idx] = float(val)
                self.data.qpos[qpos_idx[local_idx]] = float(val)
            init_angles = init_joint_pos.copy()

            if self.robot_cfg.robot_type == PI_PLUS_ROBOT_TYPE:
                if len(joint_names) != len(PI_PLUS_KP_POLICY_ORDER) or len(joint_names) != len(PI_PLUS_KD_POLICY_ORDER):
                    raise RuntimeError("pi_plus kp/kd config length mismatch")
                kp = np.asarray(PI_PLUS_KP_POLICY_ORDER, dtype=np.float32)
                kd = np.asarray(PI_PLUS_KD_POLICY_ORDER, dtype=np.float32)
            else:
                kp = parse_param_for_joint_names(pref_joints, self.robot_cfg.motor_stiffness)
                kd = parse_param_for_joint_names(pref_joints, self.robot_cfg.motor_damping)
            effort = parse_param_for_joint_names(pref_joints, self.robot_cfg.motor_effort_limit)

            specs[rid] = RobotSpec(
                rid=rid,
                name=name,
                team=team,
                qpos_idx=np.asarray(qpos_idx, dtype=np.int32),
                qvel_idx=np.asarray(qvel_idx, dtype=np.int32),
                act_idx=np.asarray(act_idx, dtype=np.int32),
                act_qpos_idx=np.asarray(qpos_idx, dtype=np.int32),
                act_qvel_idx=np.asarray(qvel_idx, dtype=np.int32),
                base_qpos_adr=base_qpos_adr,
                base_qvel_adr=base_qvel_adr,
                init_joint_pos=init_joint_pos,
                init_angles=init_angles,
                filtered_dof_target=init_angles.copy(),
                target_joint_pos=init_angles.copy(),
                last_action=np.zeros(len(joint_names), dtype=np.float32),
                action_scale=action_scale.copy(),
                kp=kp,
                kd=kd,
                effort=effort,
                obs_step_dim=obs_step_dim,
                obs_history=np.zeros((obs_dim,), dtype=np.float32),
            )
        return specs

    def set_command(self, vx, vy, w, robot_id=0, timestamp=0, source="unknown"):
        if robot_id not in FIXED_ROBOT_ID_TO_NAME:
            return
        if robot_id not in self.robot_specs:
            return
        if not self._is_command_allowed(robot_id):
            return
        if self._is_robot_protected(robot_id):
            return
        vx = float(vx)
        vy = float(vy)
        w = float(w)
        if self.robot_cfg.cmd_clip is not None:
            vx_lim, vy_lim, w_lim = self.robot_cfg.cmd_clip
            vx = float(np.clip(vx, -float(vx_lim), float(vx_lim)))
            vy = float(np.clip(vy, -float(vy_lim), float(vy_lim)))
            w = float(np.clip(w, -float(w_lim), float(w_lim)))
        ts = float(timestamp) if timestamp else time.time()
        if ts < self.command_ts[robot_id]:
            return
        self.command_ts[robot_id] = ts
        self.command_buffer[robot_id] = np.array([vx, vy, w], dtype=np.float32)
        self.command_received[robot_id] = True
        self.last_msg_info = {"timestamp": ts, "id": int(robot_id), "source": str(source)}

    def _is_command_allowed(self, robot_id: int) -> bool:
        if self.referee is None:
            return True
        pm = self.referee.play_mode
        is_left = robot_id < MAX_ROBOTS_PER_TEAM
        if pm in ("KickOff_Left", "KickIn_Left", "corner_kick_left", "goal_kick_left", "free_kick_left"):
            return bool(is_left)
        if pm in ("KickOff_Right", "KickIn_Right", "corner_kick_right", "goal_kick_right", "free_kick_right"):
            return bool(not is_left)
        return True

    def _obs_for_robot(self, spec: RobotSpec, cmd_override: np.ndarray | None = None) -> np.ndarray:
        obs_scale = self.robot_cfg.obs_scale
        qpos = self.data.qpos
        qvel = self.data.qvel

        base_lin_world = qvel[spec.base_qvel_adr : spec.base_qvel_adr + 3]
        base_ang_world = qvel[spec.base_qvel_adr + 3 : spec.base_qvel_adr + 6]
        quat = qpos[spec.base_qpos_adr + 3 : spec.base_qpos_adr + 7]
        rot_wb = _quat_to_rot_world_from_body(quat)

        base_lin = (rot_wb.T @ base_lin_world).astype(np.float32) * obs_scale["base_lin_vel"]
        if self.robot_cfg.robot_type == PI_PLUS_ROBOT_TYPE:
            # Match sim2sim_pi_plus.py: angular velocity term is consumed directly.
            base_ang = base_ang_world.astype(np.float32) * obs_scale["base_ang_vel"]
        else:
            base_ang = (rot_wb.T @ base_ang_world).astype(np.float32) * obs_scale["base_ang_vel"]
        gravity = (rot_wb.T @ np.array([0.0, 0.0, -1.0], dtype=np.float32)) * obs_scale["gravity_orientation"]
        cmd_src = self.command_buffer[spec.rid] if cmd_override is None else cmd_override

        cmd = cmd_src * obs_scale["cmd"]

        joint_pos = (qpos[spec.qpos_idx] - spec.init_joint_pos).astype(np.float32) * obs_scale["joint_pos"]
        joint_vel = qvel[spec.qvel_idx].astype(np.float32) * obs_scale["joint_vel"]
        last_action = spec.last_action * obs_scale["last_action"]

        obs_terms = [base_ang, gravity, cmd, joint_pos, joint_vel, last_action]
        if self.robot_cfg.include_base_lin_vel_obs:
            obs_terms.insert(0, base_lin)
        obs_step = np.concatenate(obs_terms, axis=-1).astype(np.float32)
        obs_step = np.nan_to_num(obs_step, nan=0.0, posinf=0.0, neginf=0.0)

        if self.robot_cfg.obs_history_length <= 1:
            return np.clip(obs_step, -self.robot_cfg.obs_clip, self.robot_cfg.obs_clip)
        spec.obs_history = np.roll(spec.obs_history, shift=-spec.obs_step_dim)
        spec.obs_history[-spec.obs_step_dim :] = obs_step
        return np.clip(spec.obs_history, -self.robot_cfg.obs_clip, self.robot_cfg.obs_clip)

    def _compute_targets(self):
        self._policy_step_count += 1
        debug_rid = FIXED_ROBOT_NAME_TO_ID.get("robot_rp0")
        if debug_rid not in self.robot_specs:
            debug_rid = next(iter(self.robot_specs.keys()), None)
        debug_obs = None
        debug_act = None

        for spec in self.robot_specs.values():
            if self._is_robot_protected(spec.rid):
                spec.last_action[:] = 0.0
                spec.filtered_dof_target[:] = spec.init_angles
                spec.target_joint_pos[:] = spec.init_angles
                continue
            zero_left = self._robot_cmd_zero_frames_left.get(spec.rid, 0)
            if zero_left > 0:
                obs = self._obs_for_robot(spec, cmd_override=np.array(DEFAULT_CMD, dtype=np.float32))
                self._robot_cmd_zero_frames_left[spec.rid] = zero_left - 1
            else:
                obs = self._obs_for_robot(spec)
            with torch.inference_mode():
                obs_tensor = torch.from_numpy(obs).unsqueeze(0).to(self.policy_device)
                act = self.policy(obs_tensor).detach().cpu().numpy().squeeze().astype(np.float32)
            act = np.nan_to_num(act, nan=0.0, posinf=0.0, neginf=0.0)
            if spec.rid == debug_rid:
                debug_obs = obs.copy()
                debug_act = act.copy()
            spec.last_action[:] = act
            act_scaled = np.clip(act * spec.action_scale, ACTION_CLIP[0], ACTION_CLIP[1])
            joint_pos_action = spec.init_angles + act_scaled
            if ACTION_SMOOTH_FILTER:
                spec.filtered_dof_target[:] = spec.filtered_dof_target * 0.2 + joint_pos_action * 0.8
            else:
                spec.filtered_dof_target[:] = joint_pos_action
            spec.target_joint_pos[:] = spec.filtered_dof_target

        if (
            not self._printed_target_policy_io
            and self._policy_step_count == self._policy_print_step
            and debug_rid is not None
            and debug_obs is not None
            and debug_act is not None
        ):
            debug_name = FIXED_ROBOT_ID_TO_NAME.get(debug_rid, f"id{debug_rid}")
            np.set_printoptions(precision=6, suppress=True)
            print(f"[Policy Frame {self._policy_step_count}] robot={debug_name} input(obs): {debug_obs}")
            print(f"[Policy Frame {self._policy_step_count}] robot={debug_name} output(action): {debug_act}")
            self._printed_target_policy_io = True

    def _apply_torque(self):
        for spec in self.robot_specs.values():
            if self._is_robot_protected(spec.rid):
                self.data.ctrl[spec.act_idx] = self._startup_ctrl[spec.act_idx]
                continue
            q = self.data.qpos[spec.qpos_idx]
            qd = self.data.qvel[spec.qvel_idx]
            q = np.nan_to_num(q, nan=0.0, posinf=0.0, neginf=0.0)
            qd = np.nan_to_num(qd, nan=0.0, posinf=0.0, neginf=0.0)
            target = np.nan_to_num(spec.target_joint_pos, nan=0.0, posinf=0.0, neginf=0.0)
            tau = spec.kp * (target - q) + spec.kd * (0.0 - qd)
            tau = np.clip(tau, -spec.effort, spec.effort)
            tau = np.nan_to_num(tau, nan=0.0, posinf=0.0, neginf=0.0)
            self.data.ctrl[spec.act_idx] = tau

    def _step_once(self, counter: int) -> int:
        pre_hold_changed = self._apply_robot_protection_holds()
        if pre_hold_changed:
            mujoco.mj_forward(self.model, self.data)
        if counter % self.control_decimation == 0:
            self._compute_targets()
        self._apply_torque()
        mujoco.mj_step(self.model, self.data)
        self._update_referee(self.sim_dt)
        post_hold_changed = self._apply_robot_protection_holds()
        if post_hold_changed:
            mujoco.mj_forward(self.model, self.data)
        if (
            not np.isfinite(self.data.qpos).all()
            or not np.isfinite(self.data.qvel).all()
            or not np.isfinite(self.data.ctrl).all()
        ):
            self.reset(preserve_ball=True)
            return 0
        return counter + 1

    def _detect_ball_contact_rid(self) -> int | None:
        active: set[int] = set()
        for i in range(int(self.data.ncon)):
            c = self.data.contact[i]
            g1 = int(c.geom1)
            g2 = int(c.geom2)
            n1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g1) or ""
            n2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, g2) or ""
            if n1 != "ball" and n2 != "ball":
                continue
            other = n2 if n1 == "ball" else n1
            if "__" not in other:
                continue
            robot_name = other.split("__", 1)[0]
            rid = FIXED_ROBOT_NAME_TO_ID.get(robot_name)
            if rid is not None and rid in self.robot_specs:
                active.add(rid)
        if not active:
            return None
        return min(active)

    def _update_referee(self, dt: float):
        if self.referee is None:
            return
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "ball")
        if bid < 0:
            return
        ball_x = float(self.data.xpos[bid][0])
        ball_y = float(self.data.xpos[bid][1])
        ball_z = float(self.data.xpos[bid][2])
        active_touch = self._detect_ball_contact_rid()
        if active_touch is not None:
            self._ball_last_touch_rid = active_touch
        self.referee.update(dt, ball_x, ball_y, ball_z, active_touch)
        place = self.referee.consume_ball_place()
        if place is not None:
            self.teleport_ball(float(place[0]), float(place[1]), None)

    def _get_ball_state(self):
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "ball-root")
        if jid < 0:
            return None
        qpos_adr = int(self.model.jnt_qposadr[jid])
        qvel_adr = int(self.model.jnt_dofadr[jid])
        return {
            "qpos": self.data.qpos[qpos_adr : qpos_adr + 7].copy(),
            "qvel": self.data.qvel[qvel_adr : qvel_adr + 6].copy(),
        }

    def _get_all_robot_states(self):
        out = {}
        for rid, spec in self.robot_specs.items():
            out[rid] = {
                "base_qpos": self.data.qpos[spec.base_qpos_adr : spec.base_qpos_adr + 7].copy(),
                "base_qvel": self.data.qvel[spec.base_qvel_adr : spec.base_qvel_adr + 6].copy(),
                "joint_qpos": self.data.qpos[spec.qpos_idx].copy(),
                "joint_qvel": self.data.qvel[spec.qvel_idx].copy(),
            }
        return out

    def _restore_ball_state(self, state):
        if state is None:
            return
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "ball-root")
        if jid < 0:
            return
        qpos_adr = int(self.model.jnt_qposadr[jid])
        qvel_adr = int(self.model.jnt_dofadr[jid])
        self.data.qpos[qpos_adr : qpos_adr + 7] = state["qpos"]
        self.data.qvel[qvel_adr : qvel_adr + 6] = state["qvel"]

    def _restore_all_robot_states(self, states):
        if not states:
            return
        for rid, state in states.items():
            spec = self.robot_specs.get(rid)
            if spec is None:
                continue
            self.data.qpos[spec.base_qpos_adr : spec.base_qpos_adr + 7] = state["base_qpos"]
            self.data.qvel[spec.base_qvel_adr : spec.base_qvel_adr + 6] = state["base_qvel"]
            self.data.qpos[spec.qpos_idx] = state["joint_qpos"]
            self.data.qvel[spec.qvel_idx] = state["joint_qvel"]

    def _reset_one_robot(self, spec: RobotSpec):
        self.data.qpos[spec.base_qpos_adr : spec.base_qpos_adr + 7] = self._startup_qpos[spec.base_qpos_adr : spec.base_qpos_adr + 7]
        self.data.qvel[spec.base_qvel_adr : spec.base_qvel_adr + 6] = self._startup_qvel[spec.base_qvel_adr : spec.base_qvel_adr + 6]
        self.data.qpos[spec.qpos_idx] = self._startup_qpos[spec.qpos_idx]
        self.data.qvel[spec.qvel_idx] = self._startup_qvel[spec.qvel_idx]
        self.data.ctrl[spec.act_idx] = self._startup_ctrl[spec.act_idx]
        spec.last_action[:] = 0.0
        spec.filtered_dof_target[:] = spec.init_angles
        spec.target_joint_pos[:] = spec.init_angles

    def _hold_robot_at_reset_pose(self, spec: RobotSpec, x: float, y: float, theta: float):
        self._reset_one_robot(spec)
        self.data.qpos[spec.base_qpos_adr + 0] = float(x)
        self.data.qpos[spec.base_qpos_adr + 1] = float(y)
        self.data.qpos[spec.base_qpos_adr + 3 : spec.base_qpos_adr + 7] = _quat_from_yaw(float(theta))
        self.data.qvel[spec.base_qvel_adr : spec.base_qvel_adr + 6] = 0.0
        self.command_buffer[spec.rid] = np.array(DEFAULT_CMD, dtype=np.float32)
        self.command_ts[spec.rid] = float("-inf")
        self.command_received[spec.rid] = True

    def _is_robot_protected(self, rid: int) -> bool:
        until = self._robot_protect_until.get(rid)
        if until is None:
            return False
        if time.monotonic() < until:
            return True
        self._robot_protect_until.pop(rid, None)
        self._robot_protect_pose.pop(rid, None)
        return False

    def _apply_robot_protection_holds(self) -> bool:
        changed = False
        now = time.monotonic()
        for rid, spec in self.robot_specs.items():
            until = self._robot_protect_until.get(rid)
            if until is None:
                continue
            if now >= until:
                self._robot_protect_until.pop(rid, None)
                self._robot_protect_pose.pop(rid, None)
                continue
            pose = self._robot_protect_pose.get(rid)
            if pose is None:
                continue
            x, y, theta = pose
            self._hold_robot_at_reset_pose(spec, x, y, theta)
            changed = True
        return changed

    def reset(self, preserve_ball: bool = True, reset_referee: bool = True):
        ball_state = self._get_ball_state() if preserve_ball else None
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:] = self._startup_qpos
        self.data.qvel[:] = self._startup_qvel
        self.data.ctrl[:] = self._startup_ctrl
        if self.data.act.size > 0 and self._startup_act.size == self.data.act.size:
            self.data.act[:] = self._startup_act
        self._apply_saved_spawn_points()
        self._restore_ball_state(ball_state)
        for spec in self.robot_specs.values():
            spec.last_action[:] = 0.0
            spec.filtered_dof_target[:] = spec.init_angles
            spec.target_joint_pos[:] = spec.init_angles
        self.command_buffer = {rid: np.array(DEFAULT_CMD, dtype=np.float32) for rid in FIXED_ROBOT_ID_TO_NAME}
        self.command_ts = {rid: float("-inf") for rid in FIXED_ROBOT_ID_TO_NAME}
        self.command_received = {rid: False for rid in FIXED_ROBOT_ID_TO_NAME}
        for rid in self.robot_specs:
            self.command_received[rid] = True
        self._robot_protect_until.clear()
        self._robot_protect_pose.clear()
        self._robot_cmd_zero_frames_left.clear()
        self._ball_last_touch_rid = None
        if reset_referee and self.referee is not None:
            self.referee.reset()
        self.last_msg_info = {"timestamp": 0.0, "id": -1, "source": "unknown"}
        self._policy_step_count = 0
        self._printed_target_policy_io = False
        mujoco.mj_forward(self.model, self.data)

    def set_spawn_points(self, spawn_points: dict[str, list[float]]):
        cleaned: dict[str, list[float]] = {}
        for name, arr in spawn_points.items():
            if not isinstance(name, str) or not isinstance(arr, (list, tuple)) or len(arr) < 2:
                continue
            if name != "ball" and name not in FIXED_ROBOT_NAME_TO_ID:
                continue
            if name != "ball":
                rid = FIXED_ROBOT_NAME_TO_ID[name]
                if rid not in self.robot_specs:
                    continue
            x = float(arr[0])
            y = float(arr[1])
            theta = float(arr[2]) if len(arr) >= 3 and arr[2] is not None else 0.0
            cleaned[name] = [x, y, theta]
        self._saved_spawn_points = cleaned

    def _apply_saved_spawn_points(self):
        if not self._saved_spawn_points:
            return
        for name, arr in self._saved_spawn_points.items():
            x, y = float(arr[0]), float(arr[1])
            theta = float(arr[2]) if len(arr) >= 3 else 0.0
            if name == "ball":
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "ball-root")
                if jid < 0:
                    continue
                qpos_adr = int(self.model.jnt_qposadr[jid])
                qvel_adr = int(self.model.jnt_dofadr[jid])
                z = float(self._startup_qpos[qpos_adr + 2])
                self.data.qpos[qpos_adr + 0] = x
                self.data.qpos[qpos_adr + 1] = y
                self.data.qpos[qpos_adr + 2] = z
                self.data.qpos[qpos_adr + 3 : qpos_adr + 7] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
                self.data.qvel[qvel_adr : qvel_adr + 6] = 0.0
                continue
            rid = FIXED_ROBOT_NAME_TO_ID.get(name)
            if rid is None or rid not in self.robot_specs:
                continue
            spec = self.robot_specs[rid]
            self.data.qpos[spec.base_qpos_adr + 0] = x
            self.data.qpos[spec.base_qpos_adr + 1] = y
            self.data.qpos[spec.base_qpos_adr + 3 : spec.base_qpos_adr + 7] = _quat_from_yaw(theta)
            self.data.qvel[spec.base_qvel_adr : spec.base_qvel_adr + 6] = 0.0

    def teleport_robot(self, robot_name: str, x: float, y: float, theta: float | None):
        rid = FIXED_ROBOT_NAME_TO_ID.get(robot_name, None)
        if rid is None or rid not in self.robot_specs:
            return
        spec = self.robot_specs[rid]
        cur_theta = self._yaw_from_quat(self.data.qpos[spec.base_qpos_adr + 3 : spec.base_qpos_adr + 7])
        target_theta = cur_theta if theta is None else float(theta)
        # Dragging a robot on minimap should only reset/reposition this robot.
        self._robot_protect_pose[rid] = (float(x), float(y), target_theta)
        self._robot_protect_until[rid] = time.monotonic() + DRAG_RESET_PROTECT_SEC
        self._robot_cmd_zero_frames_left[rid] = DRAG_CMD_ZERO_POLICY_FRAMES
        self._hold_robot_at_reset_pose(spec, float(x), float(y), target_theta)
        mujoco.mj_forward(self.model, self.data)

    def teleport_ball(self, x: float, y: float, z: float | None):
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "ball-root")
        if jid < 0:
            return
        qpos_adr = int(self.model.jnt_qposadr[jid])
        qvel_adr = int(self.model.jnt_dofadr[jid])
        if z is None:
            z = float(self.data.qpos[qpos_adr + 2])
        self.data.qpos[qpos_adr + 0] = float(x)
        self.data.qpos[qpos_adr + 1] = float(y)
        self.data.qpos[qpos_adr + 2] = float(z)
        self.data.qpos[qpos_adr + 3 : qpos_adr + 7] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.data.qvel[qvel_adr : qvel_adr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _yaw_from_quat(self, quat_wxyz: np.ndarray) -> float:
        qw, qx, qy, qz = quat_wxyz
        return float(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))

    def state_for_web(self) -> dict:
        states = {}
        for rid, name in FIXED_ROBOT_ID_TO_NAME.items():
            if rid in self.robot_specs:
                spec = self.robot_specs[rid]
                x = float(self.data.qpos[spec.base_qpos_adr + 0])
                y = float(self.data.qpos[spec.base_qpos_adr + 1])
                z = float(self.data.qpos[spec.base_qpos_adr + 2])
                quat = self.data.qpos[spec.base_qpos_adr + 3 : spec.base_qpos_adr + 7]
                cmd = self.command_buffer.get(rid, np.array(DEFAULT_CMD, dtype=np.float32))
                states[name] = {
                    "x": x,
                    "y": y,
                    "z": z,
                    "yaw": self._yaw_from_quat(quat),
                    "active": True,
                    "team": spec.team,
                    "cmd_vel": [float(cmd[0]), float(cmd[1]), float(cmd[2])],
                }
            else:
                states[name] = {
                    "x": 100.0 + rid,
                    "y": 100.0,
                    "z": 0.0,
                    "yaw": 0.0,
                    "active": False,
                    "team": "red" if rid < MAX_ROBOTS_PER_TEAM else "blue",
                    "cmd_vel": [0.0, 0.0, 0.0],
                }

        ball_x, ball_y, ball_z = 0.0, 0.0, 0.11
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "ball")
        if bid >= 0:
            ball_x = float(self.data.xpos[bid][0])
            ball_y = float(self.data.xpos[bid][1])
            ball_z = float(self.data.xpos[bid][2])
        states["ball"] = {"x": ball_x, "y": ball_y, "z": ball_z, "yaw": 0.0, "active": True, "team": "none"}
        if self.referee is not None:
            states["_game"] = self.referee.game_state_dict()
        states["_last_msg"] = dict(self.last_msg_info)
        return states

    def state_for_zmq(self) -> dict:
        robots = []
        for rid in self.active_robot_ids:
            spec = self.robot_specs[rid]
            x = float(self.data.qpos[spec.base_qpos_adr + 0])
            y = float(self.data.qpos[spec.base_qpos_adr + 1])
            quat = self.data.qpos[spec.base_qpos_adr + 3 : spec.base_qpos_adr + 7]
            robots.append(
                {
                    "id": rid,
                    "name": spec.name,
                    "x": x,
                    "y": y,
                    "theta": self._yaw_from_quat(quat),
                    "team": spec.team,
                }
            )

        ball_x, ball_y, ball_z = 0.0, 0.0, 0.11
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "ball")
        if bid >= 0:
            ball_x, ball_y, ball_z = (float(v) for v in self.data.xpos[bid])

        out = {"robots": robots, "ball": {"x": ball_x, "y": ball_y, "z": ball_z}}
        if self.referee is not None:
            out["gamecontroller"] = self.referee.game_state_dict()
        return out

    def _set_camera_eye_lookat(self, eye: tuple[float, float, float], lookat: tuple[float, float, float]):
        if self._web_camera is None:
            return
        eye_vec = np.asarray(eye, dtype=np.float32)
        look_vec = np.asarray(lookat, dtype=np.float32)
        d = eye_vec - look_vec
        dist = float(np.linalg.norm(d))
        if dist < 1e-6:
            return
        azimuth = np.degrees(np.arctan2(d[1], d[0]))
        elevation = -np.degrees(np.arcsin(np.clip(d[2] / dist, -1.0, 1.0)))
        self._web_camera.lookat[:] = look_vec
        self._web_camera.distance = dist
        self._web_camera.azimuth = float(azimuth)
        self._web_camera.elevation = float(elevation)

    def _apply_camera_preset(self, preset: str):
        presets = {
            "Top": ((0.0, 0.0, 18.0), (0.0, 0.0, 0.8)),
            # Mirror side camera so field orientation matches the mini-map (red left, blue right).
            "Side": ((0.0, 12.0, 6.0), (0.0, 0.0, 0.8)),
            "Diagonal": ((-10.0, -10.0, 10.0), (0.0, 0.0, 0.8)),
            "Goal_Left": ((-8.0, 0.0, 3.0), (0.0, 0.0, 0.9)),
            "Goal_Right": ((8.0, 0.0, 3.0), (0.0, 0.0, 0.9)),
        }
        if preset in presets:
            eye, lookat = presets[preset]
            self._set_camera_eye_lookat(eye, lookat)

    def _safe_create_renderer(self, width: int, height: int):
        candidates = [(width, height), (640, 480), (480, 360), (320, 240)]
        last_err = None
        for w, h in candidates:
            try:
                if w <= 0 or h <= 0:
                    continue
                renderer = mujoco.Renderer(self.model, width=w, height=h)
                if (w, h) != (width, height):
                    print(f"[MujocoWebView] Renderer fallback resolution: {w}x{h}")
                return renderer
            except Exception as e:
                last_err = e
        print(f"[MujocoWebView] Renderer unavailable, running without video stream: {last_err}")
        return None

    def _apply_web_commands(self, cmds, counter: int) -> tuple[int, bool]:
        reset_triggered = False
        if cmds.spawn_points is not None:
            self.set_spawn_points(cmds.spawn_points)
        if cmds.velocity_cmds is not None:
            for name, vx, vy, wz in cmds.velocity_cmds:
                rid = FIXED_ROBOT_NAME_TO_ID.get(name, None)
                if rid is None:
                    continue
                self.set_command(float(vx), float(vy), float(wz), robot_id=rid, timestamp=time.time(), source="webview")
        if cmds.reset_env:
            # Reset robots/ball/runtime state but keep current referee state.
            self.reset(preserve_ball=False, reset_referee=False)
            counter = 0
            reset_triggered = True
        if cmds.restart_match:
            # Restart full match state: reset robots, ball, and referee.
            self.reset(preserve_ball=False, reset_referee=True)
            counter = 0
            reset_triggered = True
        if cmds.viewer_point is not None and self._web_camera is not None:
            look = tuple(float(x) for x in self._web_camera.lookat)
            eye = tuple(float(x) for x in cmds.viewer_point)
            self._set_camera_eye_lookat(eye, look)
        if cmds.viewer_look_at is not None and self._web_camera is not None:
            d = float(self._web_camera.distance)
            az = np.radians(float(self._web_camera.azimuth))
            el = np.radians(float(self._web_camera.elevation))
            cur_look = np.array(self._web_camera.lookat, dtype=np.float32)
            eye = (
                cur_look[0] + d * np.cos(el) * np.cos(az),
                cur_look[1] + d * np.cos(el) * np.sin(az),
                cur_look[2] + d * np.sin(el),
            )
            look = tuple(float(x) for x in cmds.viewer_look_at)
            self._set_camera_eye_lookat(eye, look)
        if cmds.camera_preset is not None:
            self._apply_camera_preset(cmds.camera_preset)
        if cmds.teleport_cmd is not None:
            name, x, y, z, theta = cmds.teleport_cmd
            if name in FIXED_ROBOT_NAME_TO_ID:
                self.teleport_robot(name, x, y, None if theta is None else float(theta))
                counter = 0
                reset_triggered = True
            elif name == "ball":
                # Teleport ball directly so robot pose/velocity/control state are unaffected.
                self.teleport_ball(x, y, None if z is None else float(z))
        return counter, reset_triggered

    def zmq_loop(self, port: int, webview: MujocoLabWebView | None, web_fps: int, width: int, height: int):
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind(f"tcp://*:{port}")
        print(f"[MujocoZMQ] Bound to tcp://*:{port}")

        renderer = None
        frame_interval = 1.0 / max(1, web_fps)
        next_frame_time = time.time()
        state_emit_interval = 1.0 / max(1, web_fps)
        next_state_emit_time = time.time()
        if webview is not None:
            renderer = self._safe_create_renderer(width=width, height=height)
            self._web_camera = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(self._web_camera)
            self._web_camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            self._apply_camera_preset("Diagonal")

        counter = 0
        try:
            while True:
                step_start = time.time()
                reset_triggered = False
                if webview is not None:
                    cmds = webview.poll_commands()
                    counter, reset_triggered = self._apply_web_commands(cmds, counter)

                flags = zmq.NOBLOCK if webview is not None else 0
                got_msg = False
                msg = None
                try:
                    msg = socket.recv_json(flags=flags)
                    got_msg = True
                except zmq.Again:
                    pass

                if got_msg and msg is not None:
                    client_ts = msg.get("timestamp", 0)
                    msg_source = msg.get("source", "unknown")
                    if "commands" in msg and isinstance(msg["commands"], list):
                        for item in msg["commands"]:
                            if not isinstance(item, dict):
                                continue
                            c = item.get("cmd", [0.0, 0.0, 0.0])
                            rid = int(item.get("id", 0))
                            ts = item.get("timestamp", client_ts)
                            src = item.get("source", msg_source)
                            if isinstance(c, (list, tuple)) and len(c) >= 3:
                                self.set_command(float(c[0]), float(c[1]), float(c[2]), robot_id=rid, timestamp=ts, source=src)
                    else:
                        c = msg.get("cmd", [0.0, 0.0, 0.0])
                        rid = int(msg.get("id", 0))
                        if isinstance(c, (list, tuple)) and len(c) >= 3:
                            self.set_command(float(c[0]), float(c[1]), float(c[2]), robot_id=rid, timestamp=client_ts, source=msg_source)

                    if not reset_triggered:
                        counter = self._step_once(counter)
                        step_latency = time.time() - step_start
                    else:
                        step_latency = 0.0
                    response = {
                        "state": self.state_for_zmq(),
                        "sim_timestamp": time.time(),
                        "step_latency": step_latency,
                        "ack_timestamp": client_ts,
                    }
                    socket.send_json(response)
                elif webview is not None and not reset_triggered:
                    counter = self._step_once(counter)

                if webview is not None:
                    now = time.time()
                    if renderer is not None and now >= next_frame_time:
                        renderer.update_scene(self.data, camera=self._web_camera)
                        frame = renderer.render()
                        webview.emit_frame(frame)
                        next_frame_time = now + frame_interval
                    if now >= next_state_emit_time:
                        webview.emit_robot_states(self.state_for_web())
                        next_state_emit_time = now + state_emit_interval

                wait_time = self.model.opt.timestep - (time.time() - step_start)
                if wait_time > 0:
                    time.sleep(wait_time)
        finally:
            socket.close()
            context.term()
            if renderer is not None:
                renderer.close()


def run_sim(args: RuntimeArgs, template_dir: Path):
    sim = MultiRobotMujocoSim(args)
    webview = None
    if args.webview:
        webview = MujocoLabWebView(template_dir=template_dir)
        webview.start(port=args.webview_port)
        print(f"[MujocoWebView] Started at http://localhost:{args.webview_port}")

    if not args.zmq:
        raise ValueError("ZMQ is required in simplified runner. Use default --zmq.")

    sim.zmq_loop(
        port=args.port,
        webview=webview,
        web_fps=args.web_fps,
        width=args.web_width,
        height=args.web_height,
    )
