import gym
import numpy as np
from io import BytesIO
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from PIL import Image
import base64
import os
import threading

from isaaclab.envs.ui import ViewportCameraController

from labWebView import LABWEBVIEW_TEMPLATE_DIR

app = Flask(__name__, template_folder=LABWEBVIEW_TEMPLATE_DIR)

socketio = SocketIO(app, cors_allowed_origins="*")

from dataclasses import dataclass

@dataclass
class MsgBuffer:
    next_env: bool
    reset_env: bool = False
    switch_debug: bool = False
    cur_debug = False
    env_id: int = -1
    v_cmd: tuple = None
    viewer_point = None
    viewer_look_at = None
    
msg_buffer = MsgBuffer(False)

@app.route('/')
def index():
    return render_template("index.html")

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    
@socketio.on('next_env')
def handle_next_env():
    print("[Web Viewer] Received next_env trigger from frontend.")
    msg_buffer.next_env = True

@socketio.on('reset_env')
def handle_reset_env():
    print("[Web Viewer] Received reset_env trigger from frontend.")
    msg_buffer.reset_env = True

@socketio.on('switch_debug')
def handle_switch_debug():
    print("[Web Viewer] Received switch_debug trigger from frontend.")
    msg_buffer.switch_debug = True
    
@socketio.on('set_env_id')
def handle_set_env_id(data):
    env_id = data.get('id')
    print(f"[Web Viewer] Received env_id from frontend: {env_id}")
    msg_buffer.env_id = env_id

@socketio.on("set_velocity_range")
def handle_velocity_command(data):
    vx = data.get("vx", [0, 0])
    vy = data.get("vy", [0, 0])
    vz = data.get("vz", [0, 0])
    print(f"Received velocity range:\n"
          f"  X: {vx}\n"
          f"  Y: {vy}\n"
          f"  Z: {vz}")
    msg_buffer.v_cmd = (vx, vy, vz)
    
@socketio.on("set_viewer_point")
def handle_velocity_command(data):
    point = data.get("point", [3, 3, 1])
    msg_buffer.viewer_point = point
    
@socketio.on("set_viewer_look_at")
def handle_velocity_command(data):
    point = data.get("point", [3, 3, 1])
    msg_buffer.viewer_look_at = point

def run_flask_server(port):
    socketio.run(app, host='0.0.0.0', port=port, use_reloader=False, debug=False)


class RenderEnvWrapper(gym.Wrapper):
    def __init__(self, env):
        super(RenderEnvWrapper, self).__init__(env)
        self.socketio = socketio
        self.msg_buffer = msg_buffer
        # Safely access viewport_camera_controller
        self.controller = getattr(self.env.unwrapped, "viewport_camera_controller", None)
        if self.controller is not None:
            self.cfg = self.controller.cfg
        else:
            print("[RenderEnvWrapper] Warning: viewport_camera_controller is None. Camera control disabled.")
            self.cfg = None

    def _web_prev_step(self):
        if msg_buffer.env_id >= 0:
            if msg_buffer.env_id < self.env.num_envs:
                self._set_viewer_env(msg_buffer.env_id)
            msg_buffer.env_id = -1
        elif msg_buffer.next_env:
            print("next env")
            self._set_viewer_env((self.cfg.env_index + 1) % self.env.num_envs)
            msg_buffer.next_env = False
        elif msg_buffer.switch_debug:
            print("switch_debug")
            msg_buffer.cur_debug = not msg_buffer.cur_debug
            self.env.command_manager.set_debug_vis(msg_buffer.cur_debug)
            msg_buffer.switch_debug = False
        elif msg_buffer.v_cmd is not None:
            self._set_v_cmd(msg_buffer.v_cmd)
            msg_buffer.v_cmd = None
        elif msg_buffer.viewer_point is not None:
            self._set_viewer_point(msg_buffer.viewer_point)
            msg_buffer.viewer_point = None
        elif msg_buffer.viewer_look_at is not None:
            self._set_viewer_look_at(msg_buffer.viewer_look_at)
            msg_buffer.viewer_look_at = None

    def step(self, *args, **kwargs):
        self._web_prev_step()
        self.render(mode='rgb_array')
        return self.env.step(*args, **kwargs)

    def render(self, mode='rgb_array'):
        if mode == 'rgb_array':
            img = self.env.render()
            if img is None:
                # Limit warning frequency
                if not hasattr(self, "_last_warning_time"): self._last_warning_time = 0
                import time
                if time.time() - self._last_warning_time > 5.0:
                    print(f"Warning: No image data returned from the environment (throttled)")
                    self._last_warning_time = time.time()
                return

            if isinstance(img, list):
                img = img[-1]
            
            # Debug: Print first time we get an image
            if not hasattr(self, "_first_frame_logged"):
                print(f"[RenderEnvWrapper] Got first frame! Shape: {img.shape}, Type: {type(img)}")
                self._first_frame_logged = True

            img = Image.fromarray(img)
            byte_io = BytesIO()
            img.save(byte_io, format='JPEG', quality=30)
            byte_io.seek(0)
            img_data = byte_io.read()

            img_base64 = base64.b64encode(img_data).decode('utf-8')
            self.socketio.emit('new_frame', {'image': img_base64})
        return

    def web_run(self, port=5811):
        flask_thread = threading.Thread(target=run_flask_server, args=[port])
        flask_thread.start()

    def _set_viewer_env(self, idx):
        if self.controller is None: return
        self.controller.set_view_env_index(idx)

    def _set_v_cmd(self, v_cmd):
        vx, vy, vz = v_cmd
            
        try:
            for tar in [vx, vy, vz]:
                a, b = tar
                assert isinstance(a, float) or isinstance(a, int), "not valid type"
                assert isinstance(b, float) or isinstance(b, int), "not valid type"
                assert b >= a
                
            cmd_term = self.env.command_manager.get_term("base_velocity")
            cmd_term.cfg.ranges.lin_vel_x = vx
            cmd_term.cfg.ranges.lin_vel_y = vy
            cmd_term.cfg.ranges.ang_vel_z = vz
        except Exception as e:
            print(repr(e))
            
    def _set_viewer_point(self, point):
        x, y, z = point
        if self.controller is not None:
            self.controller.update_view_location(eye=(x, y, z))
        else:
            # Fallback: Move camera prim directly
            try:
                from pxr import Gf, UsdGeom
                import omni.usd
                
                stage = omni.usd.get_context().get_stage()
                # Try default camera path for Isaac Sim
                cam_path = "/OmniverseKit_Persp"
                if not stage.GetPrimAtPath(cam_path):
                     print(f"[RenderEnvWrapper] Default camera {cam_path} not found. Trying /World/Camera")
                     cam_path = "/World/Camera"
                
                prim = stage.GetPrimAtPath(cam_path)
                if prim.IsValid():
                    # We need to set translation. Rotation is harder without "look at", 
                    # but let's assume usage of set_viewer_look_at for that.
                    # Actually, update_view_location(eye=...) usually keeps same orientation or lookat?
                    # ViewportCameraController.update_view_location sets both eye and lookat.
                    # Here we only have eye. 
                    # Let's just set translation for now.
                    xform_api = UsdGeom.XformCommonAPI(prim)
                    xform_api.SetTranslate((x, y, z))
                    print(f"[RenderEnvWrapper] Set camera pos to {(x,y,z)}")
                else:
                    print(f"[RenderEnvWrapper] Camera prim not found at {cam_path}")
            except Exception as e:
                print(f"[RenderEnvWrapper] Failed to set camera pose: {e}")
        
    def _set_viewer_look_at(self, point):
        # This is harder to implement manually via USD without lookAt helper math,
        # but we can try basic or skip if too complex for quick fix.
        # Isaac Lab ViewportCameraController handles this nicely.
        # For now, let's print simple warning or try if controller is None.
        if self.controller is not None:
             x, y, z = point
             self.controller.update_view_location(lookat=(x, y, z))
        else:
             print("[RenderEnvWrapper] 'Look At' not implemented for headless fallback yet.")