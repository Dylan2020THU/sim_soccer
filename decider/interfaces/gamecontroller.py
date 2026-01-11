# interfaces/gamecontroller.py
#
#   @description:   Utilities to handle game controller data from ZMQ
#   @interfaces:
#       1. class GameController

import logging
import time

# Constants (Copied from mos-brain-k1/decider/client/receiver_new.py)
MAX_NUM_PLAYERS = 20

# SPL Team Colors
TEAM_BLUE = 0
TEAM_RED = 1
TEAM_YELLOW = 2
TEAM_BLACK = 3
TEAM_WHITE = 4
TEAM_GREEN = 5
TEAM_ORANGE = 6
TEAM_PURPLE = 7
TEAM_BROWN = 8
TEAM_GRAY = 9

TEAM_COLOR_NAMES = {
    TEAM_BLUE: "BLUE",
    TEAM_RED: "RED",
    TEAM_YELLOW: "YELLOW",
    TEAM_BLACK: "BLACK",
    TEAM_WHITE: "WHITE",
    TEAM_GREEN: "GREEN",
    TEAM_ORANGE: "ORANGE",
    TEAM_PURPLE: "PURPLE",
    TEAM_BROWN: "BROWN",
    TEAM_GRAY: "GRAY"
}

# Penalty Types
PENALTY_NONE = 0
PENALTY_SPL_ILLEGAL_BALL_CONTACT = 1
PENALTY_SPL_PLAYER_PUSHING = 2
PENALTY_SPL_ILLEGAL_MOTION_IN_SET = 3
PENALTY_SPL_INACTIVE_PLAYER = 4
PENALTY_SPL_ILLEGAL_POSITION = 5
PENALTY_SPL_LEAVING_THE_FIELD = 6
PENALTY_SPL_REQUEST_FOR_PICKUP = 7
PENALTY_SPL_LOCAL_GAME_STUCK = 8
PENALTY_SPL_ILLEGAL_POSITION_IN_SET = 9
PENALTY_SPL_PLAYER_STANCE = 10
PENALTY_SPL_ILLEGAL_MOTION_IN_STANDBY = 11
PENALTY_SUBSTITUTE = 14
PENALTY_MANUAL = 15

PENALTY_NAMES = {
    PENALTY_NONE: "NONE",
    PENALTY_SPL_ILLEGAL_BALL_CONTACT: "ILLEGAL_BALL_CONTACT",
    PENALTY_SPL_PLAYER_PUSHING: "PLAYER_PUSHING",
    PENALTY_SPL_ILLEGAL_MOTION_IN_SET: "ILLEGAL_MOTION_IN_SET",
    PENALTY_SPL_INACTIVE_PLAYER: "INACTIVE_PLAYER",
    PENALTY_SPL_ILLEGAL_POSITION: "ILLEGAL_POSITION",
    PENALTY_SPL_LEAVING_THE_FIELD: "LEAVING_THE_FIELD",
    PENALTY_SPL_REQUEST_FOR_PICKUP: "REQUEST_FOR_PICKUP",
    PENALTY_SPL_LOCAL_GAME_STUCK: "LOCAL_GAME_STUCK",
    PENALTY_SPL_ILLEGAL_POSITION_IN_SET: "ILLEGAL_POSITION_IN_SET",
    PENALTY_SPL_PLAYER_STANCE: "PLAYER_STANCE",
    PENALTY_SPL_ILLEGAL_MOTION_IN_STANDBY: "ILLEGAL_MOTION_IN_STANDBY",
    PENALTY_SUBSTITUTE: "SUBSTITUTE",
    PENALTY_MANUAL: "MANUAL"
}

# Game States
STATE_INITIAL = 0
STATE_READY = 1
STATE_SET = 2
STATE_PLAYING = 3
STATE_FINISHED = 4
STATE_STANDBY = 5

STATE_NAMES = {
    STATE_INITIAL: "STATE_INITIAL",
    STATE_READY: "STATE_READY",
    STATE_SET: "STATE_SET",
    STATE_PLAYING: "STATE_PLAYING",
    STATE_FINISHED: "STATE_FINISHED",
    STATE_STANDBY: "STATE_STANDBY"
}


class GameController:
    def __init__(self, agent):
        self.agent = agent
        self.logger = agent.get_logger()
        
        # Get config
        config = agent.get_config()
        self.team = config.get("team_id", 0)
        self.player = config.get("id", 1) - 1 # 0-indexed in array matches ID-1 usually
        
        self.game_state = "STATE_INITIAL"
        self.kicking_team = None
        self.secondary_state = None
        self.placement = None
        self.kick_off = False
        
        self.penalized_time = 0
        self.penalty = 0
        self.team_color = TEAM_BLUE
        self.team_color_name = "BLUE"
        self.player_penalty_name = "NONE"
        self.can_kick = True # Derived logic might be needed

    def update(self, game_data: dict):
        """
        Update GameController state from dictionary (parsed from ZMQ JSON).
        Expected structure is close to the GameState struct but in dict form.
        """
        if not game_data:
            return

        try:
            # Map integer state to string if needed, or keep as is.
            # The logic in decider.py expects strings like 'STATE_PLAYING'
            state_int = game_data.get('state', STATE_INITIAL)
            self.game_state = STATE_NAMES.get(state_int, "STATE_INITIAL")
            
            self.kicking_team = game_data.get('kicking_team', 0)
            self.secondary_state = game_data.get('secondary_state', None) # Might need mapping if int
            self.placement = game_data.get('drop_in_team', None) # 'set_play' or something similar? verification needed
            
            # Find our team
            teams = game_data.get('teams', [])
            our_team_data = None
            
            # Simple heuristic: if we have team numbers, match them.
            # If ZMQ sends just a list, we might need to know which index is us.
            # Adjust based on actual server implementation. 
            # Assuming teams[0] is Blue, teams[1] is Red for now if no team_number provided, or check number.
            
            if len(teams) > 0:
                # Try to find by team number
                for t in teams:
                     if t.get('team_number') == self.team:
                         our_team_data = t
                         break
                
                # Fallback
                if not our_team_data and len(teams) > self.team: 
                     # If team_id is 0 or 1 and matches index
                     our_team_data = teams[self.team]

            if our_team_data:
                self.team_color = our_team_data.get('field_player_colour', TEAM_BLUE)
                self.team_color_name = TEAM_COLOR_NAMES.get(self.team_color, "UNKNOWN")
                
                players = our_team_data.get('players', [])
                if 0 <= self.player < len(players):
                    p_info = players[self.player]
                    self.penalty = p_info.get('penalty', 0)
                    self.penalized_time = p_info.get('secs_till_unpenalized', 0)
                    self.player_penalty_name = PENALTY_NAMES.get(self.penalty, "UNKNOWN")
            
            # Determine if we can kick (simple logic for now, enhance as needed)
            self.can_kick = (self.kicking_team == self.team) or (self.game_state == "STATE_PLAYING")
            
            # self.debug_print()

        except Exception as e:
            self.logger.error(f"Error updating GameController: {e}")

    def debug_print(self):
        """记录当前比赛状态信息"""
        # timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        # log_separator = "=" * 50
        
        # self.logger.debug(log_separator)
        # self.logger.debug(f"[GameController] Game Status")
        # self.logger.debug(log_separator)
        # self.logger.debug(f"Game State      : {self.game_state}")
        # self.logger.debug(f"Kicking Team    : {self.kicking_team}")
        # self.logger.debug(f"Penalized Time  : {self.penalized_time}s")
        # self.logger.debug(f"Team Color      : {self.team_color_name}")
        # self.logger.debug(f"Player Penalty  : {self.player_penalty_name}")
        # self.logger.debug(log_separator)
        pass
