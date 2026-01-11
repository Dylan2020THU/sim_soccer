#!/usr/bin/env python3

import json
import threading
import time
import numpy as np
import logging
import os
import sys

# Add path to find logic package if needed
# assuming running from mos-brain/decider/server or mos-brain/decider
# mos-brain/decider/server -> mos-brain/decider
DECIDER_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if DECIDER_PATH not in sys.path:
    sys.path.append(DECIDER_PATH)

from logic.strategy_statemachines.defend_ball_state_machine import DefendBallStateMachine
from logic.strategy_statemachines.dribble_ball_state_machine import DribbleBallStateMachine
from logic.strategy_statemachines.shoot_ball_state_machine import ShootBallStateMachine
from logic.strategy_statemachines.attack_state_machine import StateMachine

# Define command constants
COMMANDS = {
    "dribble": "dribble",
    "forward": "forward",
    "stop": "stop",
    "find_ball": "find_ball",
    "chase_ball": "chase_ball",
    "shoot": "kick",
    "go_back_to_field": "go_back_to_field",
}

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler()
    ]
)

class StrategyServer:
    def __init__(self, ip="127.0.0.1", port="5555"):
        # Initialize the agent
        if not hasattr(self, 'logger'):
            self.logger = logging.getLogger(self.__class__.__name__)

        self.roles_to_id = {
            "forward_1": 1,
            "forward_2": 2,
            "defender_1": 3,
            "goalkeeper": 4,
        }

        # Robots data structure
        self.robots_data = {}
        # Pre-fill
        for role, robot_id in self.roles_to_id.items():
            self.robots_data[robot_id] = {
                "last_seen": None,
                "status": "disconnected",
                "data": {}
            }

        # Networking (ZMQ)
        # In simulation, we might connect to the SimServer via ZMQ
        # similar to SimClient, but requesting global info / sending global cmds?
        # For now, we stub this or assume a SimStrategyClient exists.
        # self.client = SimStrategyClient(ip, port) 
        self.client = None # Placeholder

        self._ball_pos = (0, 0)
        self.state = "stop"
        self.exit_flag = False

        self._init_state_machine()

    def _init_state_machine(self):
        self.state_machine = StateMachine(self)
        self.defend_ball_state_machine = DefendBallStateMachine(self)
        self.dribble_ball_state_machine = DribbleBallStateMachine(self)
        self.shoot_ball_state_machine = ShootBallStateMachine(self)

    def publish_command(self, player_id, cmd, data={}):
        """
        Publish a command via ZMQ (Simulated).
        """
        cmd_str = COMMANDS.get(cmd, cmd)
        self.logger.info(f"[Strategy] Publish to {player_id}: {cmd_str} (Data: {data})")
        
        # TODO: Implement actual ZMQ send to SimServer or directly to robots if architecture allows.
        # In this migration, we primarily setup the logic structure.
        pass

    def get_ball_pos(self):
        # Calculate average ball pos from robots_data
        ball_x = 0
        ball_y = 0
        count = 0
        for r_id, r_info in self.robots_data.items():
            if r_info.get("status") == "connected" and r_info.get("data", {}).get("if_ball"):
                ball_x += r_info["data"].get("ballx", 0)
                ball_y += r_info["data"].get("bally", 0)
                count += 1
        if count > 0:
            return (ball_x / count, ball_y / count)
        return (0, 0)

    @property
    def get_if_ball(self):
        for r_id, r_info in self.robots_data.items():
             if r_info.get("status") == "connected" and r_info.get("data", {}).get("if_ball"):
                 return True
        return False
    
    # ... (Other logic methods adapted from decider_server_new.py) ...
    
    def ball_in_backcourt(self):
        ball = self.get_ball_pos()
        return ball[1] < -1 # Meter conversion?
        
    def ball_in_midcourt(self):
        ball = self.get_ball_pos()
        return -1 <= ball[1] <= 1

    def ball_in_frontcourt(self):
        ball = self.get_ball_pos()
        return ball[1] > 1

    def run_defend_ball(self):
        self.defend_ball_state_machine.run()

    def run_dribble_ball(self):
        self.dribble_ball_state_machine.run()

    def run_shoot_ball(self):
        self.shoot_ball_state_machine.run()

    def stop(self):
        for role, pid in self.roles_to_id.items():
            self.publish_command(pid, "stop")

    def run(self):
        self.logger.info("Strategy Server Running (Loop)")
        try:
             while not self.exit_flag:
                 time.sleep(0.1)
                 # Update self.robots_data from ZMQ client if available
                 # self.robots_data = self.client.get_robots_data() 
                 
                 # Core Logic
                 # self.state_machine.run_in_state1() # or some trigger
                 pass
        except KeyboardInterrupt:
            self.logger.info("Stopping...")

if __name__ == "__main__":
    server = StrategyServer()
    server.run()
