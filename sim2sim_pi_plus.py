# Copyright (c) 2021-2024, The RSL-RL Project Developers.
# All rights reserved.
# Original code is licensed under the BSD-3-Clause license.
#
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# Copyright (c) 2025-2026, The Legged Lab Project Developers.
# All rights reserved.
#
# Copyright (c) 2025-2026, The TienKung-Lab Project Developers.
# All rights reserved.
# Modifications are licensed under the BSD-3-Clause license.
#
# This file contains code derived from the RSL-RL, Isaac Lab, and Legged Lab Projects,
# with additional modifications by the TienKung-Lab Project,
# and is distributed under the BSD-3-Clause license.

import argparse
import os
import sys
import xml.etree.ElementTree as xml_et
import tempfile

import mujoco
import mujoco_viewer
import numpy as np
import torch
from pynput import keyboard
import time


import pygame
from threading import Thread
import sys
import signal
import atexit
# 全局变量
x_vel_cmd, y_vel_cmd, yaw_vel_cmd = 0.8, 0.0, 0.0
x_vel_max, y_vel_max, yaw_vel_max = 1.5, 1.0, 3.0

joystick_use = True
joystick_opened = False
exit_flag = False
joystick_thread = None
joystick = None

def cleanup_resources():
    """清理资源的函数"""
    global exit_flag, joystick_thread, joystick
    
    print("正在清理手柄资源...")
    exit_flag = True
    
    # 等待手柄线程结束
    if joystick_thread and joystick_thread.is_alive():
        joystick_thread.join(timeout=2.0)  # 最多等待2秒
        if joystick_thread.is_alive():
            print("警告：手柄线程未能正常退出")
    
    # 清理pygame资源
    if joystick:
        try:
            joystick.quit()
        except:
            pass
    
    try:
        pygame.quit()
    except:
        pass
    
    print("手柄资源清理完成")

def signal_handler(signum, frame):
    """信号处理器，用于捕获Ctrl+C等信号"""
    print(f"\n接收到信号 {signum}，正在退出...")
    cleanup_resources()
    sys.exit(0)

if joystick_use:
    # 注册清理函数
    atexit.register(cleanup_resources)
    
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    pygame.init()

    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        joystick_opened = True
        print("手柄初始化成功")
    except Exception as e:
        print(f"无法打开手柄设备: {e}")
        joystick_use = False

    def handle_joystick_input():
        global exit_flag, x_vel_cmd, y_vel_cmd, yaw_vel_cmd
        
        print("手柄输入线程启动")
        while not exit_flag:
            try:
                pygame.event.get()
                
                if joystick and joystick.get_init():
                    x_vel_cmd = -joystick.get_axis(1) * x_vel_max
                    if x_vel_cmd < 0:
                        x_vel_cmd = 0.5 * x_vel_cmd
                    y_vel_cmd = -joystick.get_axis(0) * y_vel_max
                    yaw_vel_cmd = -joystick.get_axis(3) * yaw_vel_max
                
                pygame.time.delay(100)
            except Exception as e:
                print(f"手柄输入处理错误: {e}")
                break
        
        print("手柄输入线程退出")

    # 启动手柄线程
    if joystick_opened and joystick_use:
        joystick_thread = Thread(target=handle_joystick_input, daemon=True)
        joystick_thread.start()
        print("手柄线程已启动")


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd


# ============== Terrain Generation Functions ==============

def euler_to_quat(roll, pitch, yaw):
    """Convert ZYX euler angle to quaternion"""
    cx = np.cos(roll / 2)
    sx = np.sin(roll / 2)
    cy = np.cos(pitch / 2)
    sy = np.sin(pitch / 2)
    cz = np.cos(yaw / 2)
    sz = np.sin(yaw / 2)

    return np.array(
        [
            cx * cy * cz + sx * sy * sz,
            sx * cy * cz - cx * sy * sz,
            cx * sy * cz + sx * cy * sz,
            cx * cy * sz - sx * sy * cz,
        ],
        dtype=np.float64,
    )


def rot2d(x, y, yaw):
    """2D rotation"""
    nx = x * np.cos(yaw) - y * np.sin(yaw)
    ny = x * np.sin(yaw) + y * np.cos(yaw)
    return nx, ny


def list_to_str(vec):
    """Convert list to space-separated string"""
    return " ".join(str(s) for s in vec)


def add_box_to_worldbody(worldbody, position, euler, size):
    """Add a box geometry to MuJoCo worldbody"""
    geo = xml_et.SubElement(worldbody, "geom")
    geo.attrib["pos"] = list_to_str(position)
    geo.attrib["type"] = "box"
    geo.attrib["size"] = list_to_str(0.5 * np.array(size))  # half size for mujoco
    quat = euler_to_quat(euler[0], euler[1], euler[2])
    geo.attrib["quat"] = list_to_str(quat)


def add_stairs_to_scene(model_path, init_pos=[2.0, 0.0, 0.0], yaw=0.0,
                        width=0.3, height=0.15, length=2.0, stair_nums=10):
    """
    Add stairs to an existing MuJoCo XML scene.

    Args:
        model_path: Path to the original MuJoCo XML model
        init_pos: Starting position of stairs [x, y, z]
        yaw: Rotation angle in radians
        width: Width of each step (depth)
        height: Height of each step
        length: Length of each step (perpendicular to walking direction)
        stair_nums: Number of steps

    Returns:
        Path to modified XML file with stairs
    """
    # Parse the original XML
    tree = xml_et.parse(model_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")

    if worldbody is None:
        raise ValueError("No worldbody found in MuJoCo XML")

    # Fix mesh directory path to be absolute
    compiler = root.find("compiler")
    if compiler is not None and "meshdir" in compiler.attrib:
        # Convert relative meshdir to absolute path based on original model location
        original_dir = os.path.dirname(os.path.abspath(model_path))
        mesh_dir = compiler.attrib["meshdir"]
        # If it's a relative path, make it absolute
        if not os.path.isabs(mesh_dir):
            abs_mesh_dir = os.path.normpath(os.path.join(original_dir, mesh_dir))
            compiler.set("meshdir", abs_mesh_dir)

    # Add stairs
    local_pos = [0.0, 0.0, -0.5 * height]
    for _ in range(stair_nums):
        local_pos[0] += width
        local_pos[2] += height
        x, y = rot2d(local_pos[0], local_pos[1], yaw)
        add_box_to_worldbody(
            worldbody,
            [x + init_pos[0], y + init_pos[1], local_pos[2]],
            [0.0, 0.0, yaw],
            [width, length, height]
        )

    # Save to temporary file
    temp_dir = tempfile.gettempdir()
    temp_path = os.path.join(temp_dir, "pi_plus_with_stairs.xml")
    tree.write(temp_path)

    print(f"[INFO] Generated stairs terrain: {stair_nums} steps, height={height}m, width={width}m")
    print(f"[INFO] Stairs position: x={init_pos[0]}, y={init_pos[1]}, yaw={np.degrees(yaw):.1f}°")

    return temp_path


def add_slope_to_scene(model_path, init_pos=[2.0, 0.0, 0.0], yaw=0.0,
                       slope_angle=30.0, slope_length=3.0, slope_width=2.0,
                       slope_type="up"):
    """
    Add a slope to an existing MuJoCo XML scene.

    Args:
        model_path: Path to the original MuJoCo XML model
        init_pos: Starting position of slope [x, y, z]
        yaw: Rotation angle in radians (orientation of slope)
        slope_angle: Slope angle in degrees (e.g., 30 for 30-degree slope)
        slope_length: Length of the slope along the incline (in meters)
        slope_width: Width of the slope (perpendicular to walking direction)
        slope_type: "up" for uphill, "down" for downhill

    Returns:
        Path to modified XML file with slope
    """
    # Parse the original XML
    tree = xml_et.parse(model_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")

    if worldbody is None:
        raise ValueError("No worldbody found in MuJoCo XML")

    # Fix mesh directory path to be absolute
    compiler = root.find("compiler")
    if compiler is not None and "meshdir" in compiler.attrib:
        original_dir = os.path.dirname(os.path.abspath(model_path))
        mesh_dir = compiler.attrib["meshdir"]
        if not os.path.isabs(mesh_dir):
            abs_mesh_dir = os.path.normpath(os.path.join(original_dir, mesh_dir))
            compiler.set("meshdir", abs_mesh_dir)

    # Calculate slope parameters
    slope_angle_rad = np.radians(slope_angle)
    horizontal_length = slope_length * np.cos(slope_angle_rad)
    height = slope_length * np.sin(slope_angle_rad)

    # Adjust pitch based on slope type
    # Note: In MuJoCo, negative pitch tilts the front up, positive tilts front down
    if slope_type == "down":
        pitch = slope_angle_rad  # Front tilts down for downhill
        z_center = height / 2
    else:  # up
        pitch = -slope_angle_rad  # Front tilts up for uphill
        z_center = height / 2

    # Calculate center position
    x_center = horizontal_length / 2
    x, y = rot2d(x_center, 0.0, yaw)

    # Add slope as a rotated box
    # Box size should be the actual slope length, not horizontal projection
    add_box_to_worldbody(
        worldbody,
        [x + init_pos[0], y + init_pos[1], z_center + init_pos[2]],
        [0.0, pitch, yaw],
        [slope_length, slope_width, 0.02]  # Use slope length, not horizontal projection
    )

    # Save to temporary file
    temp_dir = tempfile.gettempdir()
    temp_path = os.path.join(temp_dir, "pi_plus_with_slope.xml")
    tree.write(temp_path)

    print(f"[INFO] Generated {slope_type}hill slope terrain: angle={slope_angle}°, length={slope_length}m")
    print(f"[INFO] Slope position: x={init_pos[0]}, y={init_pos[1]}, yaw={np.degrees(yaw):.1f}°")

    return temp_path


def add_continuous_slope_to_scene(model_path, init_pos=[2.0, 0.0, 0.0], yaw=0.0,
                                  uphill_angle=30.0, uphill_length=2.0,
                                  platform_length=1.0,
                                  downhill_angle=30.0, downhill_length=2.0,
                                  slope_width=2.0):
    """
    Add a continuous slope terrain: uphill -> platform -> downhill.

    Args:
        model_path: Path to the original MuJoCo XML model
        init_pos: Starting position [x, y, z]
        yaw: Rotation angle in radians
        uphill_angle: Uphill slope angle in degrees
        uphill_length: Length of uphill slope (along the incline)
        platform_length: Length of flat platform at the top
        downhill_angle: Downhill slope angle in degrees
        downhill_length: Length of downhill slope (along the incline)
        slope_width: Width of the terrain

    Returns:
        Path to modified XML file with continuous slope
    """
    # Parse the original XML
    tree = xml_et.parse(model_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")

    if worldbody is None:
        raise ValueError("No worldbody found in MuJoCo XML")

    # Fix mesh directory path
    compiler = root.find("compiler")
    if compiler is not None and "meshdir" in compiler.attrib:
        original_dir = os.path.dirname(os.path.abspath(model_path))
        mesh_dir = compiler.attrib["meshdir"]
        if not os.path.isabs(mesh_dir):
            abs_mesh_dir = os.path.normpath(os.path.join(original_dir, mesh_dir))
            compiler.set("meshdir", abs_mesh_dir)

    # Calculate uphill parameters
    uphill_angle_rad = np.radians(uphill_angle)
    uphill_horizontal = uphill_length * np.cos(uphill_angle_rad)
    uphill_height = uphill_length * np.sin(uphill_angle_rad)

    # Add uphill slope
    # Note: In MuJoCo, negative pitch tilts the front up
    # Box size should be the actual slope length, not horizontal projection
    uphill_x_center = uphill_horizontal / 2
    x, y = rot2d(uphill_x_center, 0.0, yaw)
    add_box_to_worldbody(
        worldbody,
        [x + init_pos[0], y + init_pos[1], uphill_height / 2 + init_pos[2]],
        [0.0, -uphill_angle_rad, yaw],  # Negative for uphill
        [uphill_length, slope_width, 0.02]  # Use slope length, not horizontal projection
    )

    # Add platform at the top
    platform_x_center = uphill_horizontal + platform_length / 2
    x, y = rot2d(platform_x_center, 0.0, yaw)
    add_box_to_worldbody(
        worldbody,
        [x + init_pos[0], y + init_pos[1], uphill_height + init_pos[2]],
        [0.0, 0.0, yaw],
        [platform_length, slope_width, 0.02]
    )

    # Calculate downhill parameters
    downhill_angle_rad = np.radians(downhill_angle)
    downhill_horizontal = downhill_length * np.cos(downhill_angle_rad)
    downhill_height = downhill_length * np.sin(downhill_angle_rad)

    # Add downhill slope
    # Note: In MuJoCo, positive pitch tilts the front down
    # Box size should be the actual slope length, not horizontal projection
    downhill_x_start = uphill_horizontal + platform_length
    downhill_x_center = downhill_x_start + downhill_horizontal / 2
    downhill_z_center = uphill_height - downhill_height / 2
    x, y = rot2d(downhill_x_center, 0.0, yaw)
    add_box_to_worldbody(
        worldbody,
        [x + init_pos[0], y + init_pos[1], downhill_z_center + init_pos[2]],
        [0.0, downhill_angle_rad, yaw],  # Positive for downhill
        [downhill_length, slope_width, 0.02]  # Use slope length, not horizontal projection
    )

    # Save to temporary file
    temp_dir = tempfile.gettempdir()
    temp_path = os.path.join(temp_dir, "pi_plus_with_continuous_slope.xml")
    tree.write(temp_path)

    total_length = uphill_horizontal + platform_length + downhill_horizontal
    print(f"[INFO] Generated continuous slope terrain:")
    print(f"  - Uphill: {uphill_angle}° × {uphill_length}m → height {uphill_height:.2f}m")
    print(f"  - Platform: {platform_length}m at height {uphill_height:.2f}m")
    print(f"  - Downhill: {downhill_angle}° × {downhill_length}m")
    print(f"  - Total length: {total_length:.2f}m")
    print(f"[INFO] Terrain position: x={init_pos[0]}, y={init_pos[1]}, yaw={np.degrees(yaw):.1f}°")

    return temp_path


class SimToSimCfg:
    """Configuration class for sim2sim parameters.

    Must be kept consistent with the training configuration.
    """

    class sim:
        sim_duration = 100.0
        num_action = 20
        num_obs_per_step = 69
        actor_obs_history_length = 5
        dt = 0.002
        decimation = 10
        clip_observations = 100.0
        clip_actions = 100.0
        action_scale = 0.25



class MujocoRunner:
    """
    Sim2Sim runner that loads a policy and a MuJoCo model
    to run real-time humanoid control simulation.

    Args:
        cfg (SimToSimCfg): Configuration object for simulation.
        policy_path (str): Path to the TorchScript exported policy.
        model_path (str): Path to the MuJoCo XML model.
    """

    def __init__(self, cfg: SimToSimCfg, policy_path, model_path):
        self.cfg = cfg
        network_path = policy_path
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.model.opt.timestep = self.cfg.sim.dt
        for i in range(self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            joint_type = self.model.jnt_type[i]
            joint_type_str = {0: "free", 1: "ball", 2: "slide", 3: "hinge"}.get(joint_type, "unknown")
            print(f"  {i:2d}: {joint_name:25s} (type={joint_type_str})")

        self.policy = torch.jit.load(network_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
        self.viewer._render_every_frame = False
        self.init_variables()
        self.data.qpos[7:] = self.default_dof_pos.copy()
        self.data.qvel[6:] = np.zeros_like(self.data.qvel[6:])
        mujoco.mj_forward(self.model, self.data)

    def init_variables(self) -> None:
        """Initialize simulation variables and joint index mappings."""
        self.dt = self.cfg.sim.decimation * self.cfg.sim.dt
        self.dof_pos = np.zeros(self.cfg.sim.num_action)
        self.dof_vel = np.zeros(self.cfg.sim.num_action)
        self.action = np.zeros(self.cfg.sim.num_action)
        self.default_dof_pos = np.array(
            [-0.25, 0.0, 0.0, 0.65, -0.4, 0.0, 0.0, 0.2, 0.0, -1.2,
             -0.25, 0.0, 0.0, 0.65, -0.4, 0.0, 0.0, -0.2, 0.0, -1.2, ]
        )
        self.episode_length_buf = 0
        self.kps = [80, 80, 80, 80, 60, 60, 30, 30, 30, 30, 80, 80, 80, 80, 60, 60, 30, 30, 30, 30]
        self.kds = [1.1, 1.1, 1.1, 1.1, 1.2, 1.2, 0.6, 0.6, 0.6, 0.6, 1.1, 1.1, 1.1, 1.1, 1.2, 1.2, 0.6, 0.6, 0.6, 0.6]
        """
        mujoco joint order (floating base removed; reindexed from 0)
        0   l_hip_pitch           
        1   l_hip_roll            10  r_hip_pitch
        2   l_thigh               11  r_hip_roll
        3   l_calf                12  r_thigh
        4   l_ankle_pitch         13  r_calf
        5   l_ankle_roll          14  r_ankle_pitch
        6   l_shoulder_pitch      15  r_ankle_roll
        7   l_shoulder_roll       16  r_shoulder_pitch
        8   l_upper_arm           17  r_shoulder_roll
        9   l_elbow               18  r_upper_arm
                                  19  r_elbow
                                 
        """

        """
        isaac joint order
        0  l_hip_pitch          15 r_elbow     
        1  l_shoulder_pitch     16 l_ankle_pitch    
        2  r_hip_pitch          17 r_ankle_pitch
        3  r_shoulder_pitch     18 l_ankle_roll      
        4  l_hip_roll           19 r_ankle_roll
        5  l_shoulder_roll     
        6  r_hip_roll           
        7  r_shoulder_roll      
        8  l_thigh         
        9  l_upper_arm     
        10 r_thigh
        11 r_upper_arm
        12 l_calf
        13 l_elbow
        14 r_calf
        """

        self.isaac_to_mujoco_idx = [0,4,8,12,16,18,1,5,9,13,2,6,10,14,17,19,3,7,11,15]
        self.mujoco_to_isaac_idx = [0,6,10,16,1,7,11,17,2,8,12,18,3,9,13,19,4,14,5,15]
        # Initial command vel
        self.command_vel = np.array([0.0, 0.0, 0.0])
        self.obs_history = np.zeros(
            (self.cfg.sim.num_obs_per_step * self.cfg.sim.actor_obs_history_length,), dtype=np.float32
        )

    def get_obs(self) -> np.ndarray:
        """
        Compute current observation vector from MuJoCo sensors and internal state.

        Returns:
            np.ndarray: Normalized and clipped observation history.
        """
        self.dof_pos = self.data.qpos[7:]
        self.dof_vel = self.data.qvel[6:]

        obs = np.zeros((self.cfg.sim.num_obs_per_step,), dtype=np.float32)

 
        # Angular vel
        obs[0:3] = self.data.sensor("angular-velocity").data.astype(np.double)

        # Projected gravity
        obs[3:6] = self.quat_apply_inverse(
            self.data.sensor("orientation").data[[1, 2, 3, 0]].astype(np.double), np.array([0, 0, -1])
        )
        # Command velocity
        # self.command_vel[0] = 0
        # self.command_vel[1] = 0
        # self.command_vel[2] = 0
        self.command_vel[0] = x_vel_cmd
        self.command_vel[1] = y_vel_cmd
        self.command_vel[2] = yaw_vel_cmd
        obs[6:9] = self.command_vel

        # Dof pos
        obs[9:29] = (self.dof_pos - self.default_dof_pos)[self.mujoco_to_isaac_idx]

        # Dof vel
        obs[29:49] = self.dof_vel[self.mujoco_to_isaac_idx]

        # Action
        obs[49:69] = np.clip(self.action, -self.cfg.sim.clip_actions, self.cfg.sim.clip_actions)



        # Update observation history
        self.obs_history = np.roll(self.obs_history, shift=-self.cfg.sim.num_obs_per_step)
        self.obs_history[-self.cfg.sim.num_obs_per_step :] = obs.copy()

        return np.clip(self.obs_history, -self.cfg.sim.clip_observations, self.cfg.sim.clip_observations)



    def run(self) -> None:
        """
        Run the simulation loop with keyboard-controlled commands.
        """
        self.setup_keyboard_listener()
        self.listener.start()

        while self.data.time < self.cfg.sim.sim_duration:
            self.obs_history = self.get_obs()
            # self.action[:] = self.policy(torch.tensor(self.obs_history, dtype=torch.float32)).detach().numpy()[:21]
            # 由于obs改成没有历史了，所以这里也要改
            # 将 obs_history 转成 tensor 并加 batch 维度
            # obs_tensor = torch.tensor(self.obs_history, dtype=torch.float32)
            # print("obs_tensor shape0 : ", obs_tensor.shape)
            obs_tensor = torch.tensor(self.obs_history, dtype=torch.float32).unsqueeze(0)  # [1, obs_dim]
            # print("obs_tensor shape1 : ", obs_tensor.shape)

            # 调用 policy
            action_tensor = self.policy(obs_tensor).detach().numpy()

            # 取前20个动作
            self.action[:] = action_tensor[0, :20]

            self.action = np.clip(self.action, -self.cfg.sim.clip_actions, self.cfg.sim.clip_actions)

            actions_scaled = self.action * self.cfg.sim.action_scale
            target_dof_pos = actions_scaled[self.isaac_to_mujoco_idx] + self.default_dof_pos

            for _ in range(self.cfg.sim.decimation):
                step_start_time = time.time()

                tau = pd_control(target_dof_pos, self.data.qpos[7:], self.kps, np.zeros_like(self.kds), self.data.qvel[6:], self.kds)
                self.data.ctrl = tau
                mujoco.mj_step(self.model, self.data)
                self.viewer.render()

                elapsed = time.time() - step_start_time
                sleep_time = self.cfg.sim.dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
            self.episode_length_buf += 1
     

        self.listener.stop()
        self.viewer.close()

    def quat_apply_inverse(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """
        Rotate a vector by the inverse of a quaternion.

        Args:
            q (np.ndarray): Quaternion (x, y, z, w) format.
            v (np.ndarray): Vector to rotate.

        Returns:
            np.ndarray: Rotated vector.
        """
        q_w = q[-1]
        q_vec = q[:3]
        a = v * (2.0 * q_w**2 - 1.0)
        b = np.cross(q_vec, v) * q_w * 2.0
        c = q_vec * np.dot(q_vec, v) * 2.0

        return a - b + c


    def adjust_command_vel(self, idx: int, increment: float) -> None:
        """
        Adjust command velocity vector.

        Args:
            idx (int): Index of velocity component (0=x, 1=y, 2=yaw).
            increment (float): Value to increment.
        """
        self.command_vel[idx] += increment
        self.command_vel[idx] = np.clip(self.command_vel[idx], -1.0, 1.0)  # vel clip

    def setup_keyboard_listener(self) -> None:
        """
        Set up keyboard event listener for user control input.
        """

        def on_press(key):
            try:
                if key.char == "8":  # NumPad 8      x += 0.2
                    self.adjust_command_vel(0, 0.2)
                elif key.char == "2":  # NumPad 2      x -= 0.2
                    self.adjust_command_vel(0, -0.2)
                elif key.char == "4":  # NumPad 4      y -= 0.2
                    self.adjust_command_vel(1, -0.2)
                elif key.char == "6":  # NumPad 6      y += 0.2
                    self.adjust_command_vel(1, 0.2)
                elif key.char == "7":  # NumPad 7      yaw += 0.2
                    self.adjust_command_vel(2, -0.2)
                elif key.char == "9":  # NumPad 9      yaw -= 0.2
                    self.adjust_command_vel(2, 0.2)
            except AttributeError:
                pass

        self.listener = keyboard.Listener(on_press=on_press)


if __name__ == "__main__":
    LEGGED_LAB_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
    parser = argparse.ArgumentParser(description="Run sim2sim Mujoco controller.")
    parser.add_argument(
        "--task",
        type=str,
        default="walk",
        choices=["walk"],
    )
    parser.add_argument(
        "--policy",
        type=str,
        default=None,
        help="Path to policy.pt. If not specified, it will be set automatically based on --task",
    )
    parser.add_argument(
        "--model",
        type=str,
        default=os.path.join(LEGGED_LAB_ROOT_DIR, "legged_lab/assets/hightorque/pi_plus/pi_plus.xml"),
        help="Path to model.xml",
    )
    parser.add_argument("--duration", type=float, default=100.0, help="Simulation duration in seconds")

    # Terrain arguments
    parser.add_argument("--terrain", type=str, default=None,
                        choices=["stairs", "slope", "continuous_slope", "none"],
                        help="Type of terrain to add (default: none)")

    # Common terrain parameters
    parser.add_argument("--terrain-pos", type=float, nargs=3, default=[2.0, 0.0, 0.0],
                        help="Terrain initial position [x, y, z] (default: 2.0 0.0 0.0)")
    parser.add_argument("--terrain-yaw", type=float, default=0.0,
                        help="Terrain rotation angle in degrees (default: 0)")

    # Stairs parameters
    parser.add_argument("--stair-width", type=float, default=0.3,
                        help="Width (depth) of each stair step in meters (default: 0.3)")
    parser.add_argument("--stair-height", type=float, default=0.15,
                        help="Height of each stair step in meters (default: 0.15)")
    parser.add_argument("--stair-length", type=float, default=2.0,
                        help="Length of each stair step in meters (default: 2.0)")
    parser.add_argument("--stair-nums", type=int, default=10,
                        help="Number of stair steps (default: 10)")

    # Slope parameters
    parser.add_argument("--slope-angle", type=float, default=30.0,
                        help="Slope angle in degrees (default: 30)")
    parser.add_argument("--slope-length", type=float, default=3.0,
                        help="Length of slope along the incline in meters (default: 3.0)")
    parser.add_argument("--slope-width", type=float, default=2.0,
                        help="Width of slope in meters (default: 2.0)")
    parser.add_argument("--slope-type", type=str, default="up", choices=["up", "down"],
                        help="Slope direction: 'up' or 'down' (default: up)")

    # Continuous slope parameters
    parser.add_argument("--uphill-angle", type=float, default=30.0,
                        help="Uphill slope angle in degrees (default: 30)")
    parser.add_argument("--uphill-length", type=float, default=2.0,
                        help="Length of uphill section in meters (default: 2.0)")
    parser.add_argument("--platform-length", type=float, default=1.0,
                        help="Length of platform at the top in meters (default: 1.0)")
    parser.add_argument("--downhill-angle", type=float, default=30.0,
                        help="Downhill slope angle in degrees (default: 30)")
    parser.add_argument("--downhill-length", type=float, default=2.0,
                        help="Length of downhill section in meters (default: 2.0)")

    args = parser.parse_args()

    if args.policy is None:
        args.policy = os.path.join(LEGGED_LAB_ROOT_DIR, "Exported_policy", f"{args.task}.pt")

    if not os.path.isfile(args.policy):
        print(f"[ERROR] Policy file not found: {args.policy}")
        sys.exit(1)
    if not os.path.isfile(args.model):
        print(f"[ERROR] MuJoCo model file not found: {args.model}")
        sys.exit(1)

    print(f"[INFO] Loaded task preset: {args.task.upper()}")
    print(f"[INFO] Loaded policy: {args.policy}")
    print(f"[INFO] Loaded model: {args.model}")

    # Apply terrain modifications if requested
    model_path = args.model
    if args.terrain == "stairs":
        model_path = add_stairs_to_scene(
            model_path=args.model,
            init_pos=args.terrain_pos,
            yaw=np.radians(args.terrain_yaw),  # Convert degrees to radians
            width=args.stair_width,
            height=args.stair_height,
            length=args.stair_length,
            stair_nums=args.stair_nums
        )
    elif args.terrain == "slope":
        model_path = add_slope_to_scene(
            model_path=args.model,
            init_pos=args.terrain_pos,
            yaw=np.radians(args.terrain_yaw),
            slope_angle=args.slope_angle,
            slope_length=args.slope_length,
            slope_width=args.slope_width,
            slope_type=args.slope_type
        )
    elif args.terrain == "continuous_slope":
        model_path = add_continuous_slope_to_scene(
            model_path=args.model,
            init_pos=args.terrain_pos,
            yaw=np.radians(args.terrain_yaw),
            uphill_angle=args.uphill_angle,
            uphill_length=args.uphill_length,
            platform_length=args.platform_length,
            downhill_angle=args.downhill_angle,
            downhill_length=args.downhill_length,
            slope_width=args.slope_width
        )

    sim_cfg = SimToSimCfg()
    sim_cfg.sim.sim_duration = args.duration
    runner = MujocoRunner(
        cfg=sim_cfg,
        policy_path=args.policy,
        model_path=model_path,
    )
    runner.run()