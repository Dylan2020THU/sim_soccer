# user_entry.py
#
#   @description:   Logic entry point for mos-brain decider
#                   Game logic is separated into game() function, 
#                   loop() is kept minimal for framework stability.
#

import time
import traceback
import sys
import os

# Ensure we can find the logic package
CUR_DIR = os.path.dirname(os.path.abspath(__file__))
if CUR_DIR not in sys.path:
    sys.path.append(CUR_DIR)

from logic.sub_statemachines import chase_ball, find_ball, kick, go_back_to_field, dribble
from logic.policy_statemachines import goalkeeper

def init(agent) -> None:
    agent.get_logger().info("[UserEntry] Initializing Logic...")
    
    # Initialize State Machines
    # We attach them to agent to persist state
    agent.chase_ball_machine = chase_ball.ChaseBallStateMachine(agent)
    agent.find_ball_machine = find_ball.FindBallStateMachine(agent)
    agent.kick_machine = kick.KickStateMachine(agent)
    agent.go_back_machine = go_back_to_field.GoBackToFieldStateMachine(agent)
    agent.dribble_machine = dribble.DribbleStateMachine(agent)
    # Check if goalkeeper config exists, otherwise skip or default
    agent.goalkeeper_machine = goalkeeper.GoalkeeperStateMachine(agent)

    agent.state_machine_runners = {
        "chase_ball": agent.chase_ball_machine.run,
        "find_ball": agent.find_ball_machine.run,
        "kick": agent.kick_machine.run, # handle special kick logic if needed
        "go_back_to_field": agent.go_back_machine.run,
        "dribble": agent.dribble_machine.run,
        "stop": agent.stop,
        "goalkeeper": agent.goalkeeper_machine.run,
    }
    
    # Additional Init
    agent._last_play_time = time.time()
    agent.decider_start_time = time.time()
    agent.penalize_end_time = 0
    agent.start_walk_into_field_time = agent.get_config().get("start_walk_into_field_time", 3)
    agent.default_chase_distance = agent.get_config().get("chase",{}).get("default_chase_distance", 0.7)
    
    # Kicking params
    agent.league = agent.get_config().get("league", "kid")
    agent.dribble_to_kick = agent.get_config().get("dribble_to_kick", {}).get(agent.league, [-2.3, 2.3])
    agent.kick_to_dribble = agent.get_config().get("kick_to_dribble", {}).get(agent.league, [-1.7, 1.7])
    agent.attack_method = "dribble"
    
    # Relocalize
    agent.relocate()

def loop(agent) -> None:
    """
    Main loop function called by the agent.
    Kept minimal - delegates to game() for actual logic.
    Override game() to implement custom behavior.
    """
    try:
        # Simply delegate to game logic
        game(agent)
    except Exception as e:
        agent.get_logger().error(f"Error in user_entry loop: {e}")
        traceback.print_exc()

def game(agent) -> None:
    """
    Game logic entry point.
    """
    # Choose one of the following modes:
    # _game_logic(agent)           # Full game logic
    # _test_dribble(agent)         # Test dribble
    _test_go_back_to_field(agent)# Test go back to field
    # _test_find_ball(agent)       # Test find ball
    # _test_state_machines(agent)    # Default: Test basic state machines
    # _test_basic_interfaces(agent)  # Test basic observation & control


# ============================================================================
# Basic Interface Testing Mode
# ============================================================================

def _test_basic_interfaces(agent) -> None:
    """
    Test basic observation and control interfaces.
    Cycles through simple control tests while logging all observations.
    """
    # Initialize test state
    if not hasattr(agent, '_basic_test_state'):
        agent._basic_test_state = {
            'start_time': time.time(),
            'phase': 0,
            'phase_start': time.time(),
            'log_interval': 1.0,  # Log every 1 second
            'last_log': 0,
        }
        agent.get_logger().info("=" * 60)
        agent.get_logger().info("[BASIC TEST] Starting Basic Interface Test")
        agent.get_logger().info("=" * 60)
    
    state = agent._basic_test_state
    current_time = time.time()
    elapsed = current_time - state['start_time']
    phase_elapsed = current_time - state['phase_start']
    
    # Log observations periodically
    if current_time - state['last_log'] >= state['log_interval']:
        _log_observations(agent)
        state['last_log'] = current_time
    
    # Control test phases (each 5 seconds)
    PHASE_DURATION = 5.0
    phases = [
        ("STOP", (0, 0, 0)),
        ("FORWARD", (0.5, 0, 0)),
        ("BACKWARD", (-0.5, 0, 0)),
        ("LEFT", (0, 0.5, 0)),
        ("RIGHT", (0, -0.5, 0)),
        ("ROTATE_LEFT", (0, 0, 0.8)),
        ("ROTATE_RIGHT", (0, 0, -0.8)),
        ("DIAGONAL", (0.3, 0.2, 0.3)),
    ]
    
    # Switch phase if needed
    if phase_elapsed >= PHASE_DURATION:
        state['phase'] = (state['phase'] + 1) % len(phases)
        state['phase_start'] = current_time
        phase_name, cmd = phases[state['phase']]
        agent.get_logger().info(f"[BASIC TEST] Switching to phase: {phase_name} cmd_vel={cmd}")
    
    # Execute current phase
    phase_name, cmd = phases[state['phase']]
    agent.cmd_vel(cmd[0], cmd[1], cmd[2])


def _log_observations(agent) -> None:
    """Log all observation interface values."""
    logger = agent.get_logger()
    
    # Separator
    logger.info("-" * 50)
    logger.info("[OBS] === Observation Report ===")
    
    # Self position
    self_pos = agent.get_self_pos()
    self_yaw = agent.get_self_yaw()
    logger.info(f"[OBS] Self Pos: x={self_pos[0]:.3f}, y={self_pos[1]:.3f}")
    logger.info(f"[OBS] Self Yaw: {self_yaw:.3f} rad ({self_yaw * 180 / 3.14159:.1f} deg)")
    
    # Ball detection
    if_ball = agent.get_if_ball()
    logger.info(f"[OBS] Ball Detected: {if_ball}")
    
    if if_ball:
        ball_pos = agent.get_ball_pos()
        ball_angle = agent.get_ball_angle()
        ball_distance = agent.get_ball_distance()
        ball_in_map = agent.get_ball_pos_in_map()
        
        logger.info(f"[OBS] Ball Pos (relative): x={ball_pos[0]:.3f}, y={ball_pos[1]:.3f}")
        logger.info(f"[OBS] Ball Angle: {ball_angle:.3f} rad" if ball_angle else "[OBS] Ball Angle: None")
        logger.info(f"[OBS] Ball Distance: {ball_distance:.3f} m")
        if ball_in_map is not None:
            logger.info(f"[OBS] Ball Pos (map): x={ball_in_map[0]:.3f}, y={ball_in_map[1]:.3f}")
        
        # Close to ball check
        close = agent.get_if_close_to_ball()
        logger.info(f"[OBS] Close to Ball: {close}")
    
    # Head/Neck
    neck = agent.get_neck()
    head = agent.get_head()
    logger.info(f"[OBS] Neck: {neck:.3f}, Head: {head:.3f}")
    
    # Config check
    config = agent.get_config()
    robot_id = config.get("id", "unknown")
    logger.info(f"[OBS] Robot ID: {robot_id}")
    
    logger.info("-" * 50)

# ============================================================================
# Game Logic Implementation (separated for modularity)
# ============================================================================

def _game_logic(agent) -> None:
    """Full game state machine logic - call from game() when needed."""
    # Access Interfaces
    gamecontroller = agent.gamecontroller
    communication = agent.communication
    
    state = gamecontroller.game_state
    secondary_state = gamecontroller.secondary_state
    can_kick = gamecontroller.can_kick
    penalty = gamecontroller.penalty
    
    agent.get_logger().debug(f"GameState: {state}, Penalty: {penalty}")
    
    # 1. Handle Penalty
    if penalty != 0:
         agent.get_logger().debug(f"Penalized: {penalty}")
         agent.stop()
         agent.penalize_end_time = time.time()
         return

    # 2. Handle Non-Playing States
    if state != "STATE_PLAYING":
        agent._last_play_time = time.time()
        
    if state in ['STATE_SET', 'STATE_FINISHED', 'STATE_INITIAL', None]:
        if state is None:
            agent.decider_start_time = time.time()
        agent.stop()
        return
        
    # 3. Handle Ready
    if state == 'STATE_READY':
         agent.state_machine_runners['go_back_to_field']()
         return

    # 4. Handle Playing
    current_time = time.time()
    
    # Simplistic start delay / walk in
    if current_time - agent.decider_start_time < agent.start_walk_into_field_time:
        agent.state_machine_runners['go_back_to_field']()
        return
    elif current_time - agent.penalize_end_time < agent.start_walk_into_field_time:
         agent.state_machine_runners['go_back_to_field']()
         return
         
    # Role: Goalkeeper
    if agent.get_config().get("if_goalkeeper", False):
         agent.state_machine_runners['goalkeeper']()
         return
         
    # Kick Off handling
    if gamecontroller.kicking_team != gamecontroller.team and (current_time - agent._last_play_time < 10.0):
         if not agent.get_if_ball():
             agent.state_machine_runners['find_ball']()
         else:
             agent.state_machine_runners['chase_ball'](distance=2.0)
         return

    # Cannot kick?
    if not can_kick:
         if not agent.get_if_ball():
             agent.state_machine_runners['find_ball']()
         else:
             agent.state_machine_runners['chase_ball'](distance=1.5)
         return
         
    # 5. Autonomous play
    _handle_autonomous(agent)

def _handle_autonomous(agent):
    """Fallback autonomous logic if no external command"""
    if not agent.get_if_ball():
        agent.state_machine_runners['find_ball']()
    elif agent.get_ball_distance() > agent.default_chase_distance:
        agent.state_machine_runners['chase_ball']()
    else:
        # Dribble or Kick logic
        _handle_kick_logic(agent)

def _handle_kick_logic(agent):
    # Determine kick vs dribble based on field position
    self_pos_y = agent.get_self_pos()[1]
    
    if self_pos_y < agent.dribble_to_kick[0] or self_pos_y > agent.dribble_to_kick[1]:
        agent.attack_method = "kick"
    if self_pos_y > agent.kick_to_dribble[0] and self_pos_y < agent.kick_to_dribble[1]:
        agent.attack_method = "dribble"
        
    if agent.attack_method == "dribble":
        agent.state_machine_runners['dribble']()
    else:
        # Kick state machine
        agent.state_machine_runners['kick']()


# ============================================================================
# State Machine Testing
# ============================================================================

def _test_state_machines(agent) -> None:
    """
    Test state machines: Find Ball -> Chase Ball
    """
    if agent.get_if_ball():
        agent.state_machine_runners['chase_ball']()
    else:
        agent.state_machine_runners['find_ball']()


def _test_dribble(agent) -> None:
    """
    Test dribble state machine
    """
    agent.state_machine_runners['dribble']()


def _test_go_back_to_field(agent) -> None:
    """
    Test go_back_to_field state machine
    """
    agent.state_machine_runners['go_back_to_field']()


def _test_find_ball(agent) -> None:
    """
    Test find_ball state machine
    """
    agent.state_machine_runners['find_ball']()

