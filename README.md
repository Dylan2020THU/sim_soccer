# MOS-Brain Documentation

This repository contains the decision-making framework (`decider`) and simulation environment (`simulation`) for the MOS Multi-Agent Soccer system.

## 1. Quick Start

### Prerequisites
The decision code requires the `k1` Conda environment.
```bash
conda activate k1
```

### Step 1: Launch Simulation
Usually, the simulation is already running in a `screen` session named `sim` (or `wj`). You do NOT need to launch it manually in this case.

If you need to view the logs or check if it is running:
```bash
screen -r sim
# Detach with Ctrl+A, D
```

If it is not running, you can launch it (headless with web viewer enabled):
```bash
./simulation/scripts/launch_sim.sh --headless --webview --task Robocup-Soccer
```
The simulation provides a **Web Viewer** at:
- **URL**: `http://localhost:5811` (Port 5811)
- Use this to visualize the match.

### Step 2: Start Robot Decision

#### Option A: Start a Single Robot
To control a specific robot, use the `decider.py` script. You must specify the team color and player ID.

```bash
# Start Red Team Player 1 (ID 0)
python3 decider/decider.py --simulation --color red --id 0

# Start Blue Team Player 1 (ID 0)
python3 decider/decider.py --simulation --color blue --id 0
```
*Note: The script automatically handles the global ID mapping (e.g., Blue 0 maps to simulation ID N).*

#### Option B: Start Entire Teams
To launch all robots defined in the configuration at once:
```bash
./decider/scripts/start_team.sh
```

**Naming Convention:**
This script launches each robot in a separate `screen` session with the following naming:
- **Red Team**: `decider_red_0`, `decider_red_1`, ...
- **Blue Team**: `decider_blue_0`, `decider_blue_1`, ...

**Useful Commands:**
- **Attach to a robot's log:** `screen -r decider_red_0` (Detach with Ctrl+A, D)
- **Restart all robots:** `./decider/scripts/start_team.sh --restart`
- **Kill all robots:** `./decider/scripts/start_team.sh --kill`
- **Override robot counts:** `./decider/scripts/start_team.sh --red 2 --blue 1`

## 2. Configuration

### Simulation Configuration
Edit `simulation/config/match_config.json` to change the number of robots or field size.
```json
"teams": {
    "red": { "count": 2, ... },
    "blue": { "count": 2, ... }
}
```

### Robot & Strategy Configuration
Edit `decider/config.yaml` to tune robot parameters, thresholds, and default settings.
- **color**: Default team color if not specified in CLI.
- **id**: Default ID.

## 3. Writing Custom Strategy

The main entry point for user strategy is **`decider/user_entry.py`**.

### The `game(agent)` Loop
The `game(agent)` function is called every cycle. You should implement your high-level logic here.
```python
def game(agent):
    # Example simple logic
    if not agent.get_if_ball():
        agent.state_machine_runners['find_ball']()
    else:
        agent.state_machine_runners['chase_ball']()
```

### State Machines
We provide several built-in state machines in `decider/logic/sub_statemachines/`:
- `find_ball`: Rotates to find the ball.
- `chase_ball`: Approaches the ball.
- `dribble`: Dribbles the ball towards the goal.
- `kick`: Kicks the ball (if enabled).
- `go_back_to_field`: Returns to the field center.

Usage: `agent.state_machine_runners['state_machine_name']()`

## 4. API Reference

The `agent` object passed to `game()` provides all necessary interfaces.

### Interaction Interfaces
Source code locations:
- **Action**: `decider/interfaces/action.py`
- **Vision**: `decider/interfaces/vision.py`
- **GameController**: `decider/interfaces/gamecontroller.py`

### Coordinate Systems
- **Robot Frame**:
    - **X**: Forward (meters)
    - **Y**: Left (meters)
    - **Theta**: Counter-clockwise (radians), 0 is forward.
- **Map Frame**: 
    - **Y-Axis**: Points towards the **Opponent's Goal**.
    - **X-Axis**: Points to the Right (relative to facing opponent goal).
    - **Note**: The Blue Team's coordinate system is **mirrored** relative to the Simulator's Global coordinates, so that both teams share the same logical strategies (always attacking "Positive Y").

### Perception Interface
| Method | Description | Returns |
|--------|-------------|---------|
| `agent.get_ball_pos()` | Ball position in **Robot Frame**. | `[x, y]` or `[None, None]` |
| `agent.get_ball_distance()` | Distance to ball. | `float` (meters) |
| `agent.get_ball_angle()` | Angle to ball in **Robot Frame**. | `float` (radians) |
| `agent.get_if_ball()` | Is ball currently visible? | `bool` |
| `agent.get_self_pos()` | Robot position in **Map Frame**. | `[x, y]` |
| `agent.get_self_yaw()` | Robot orientation in **Map Frame**. | `float` (radians) |

### Action Interface
| Method | Description |
|--------|-------------|
| `agent.cmd_vel(vx, vy, vtheta)` | Send velocity command (Robot Frame). <br> `vx`: Forward speed (m/s) <br> `vy`: Lateral speed (m/s) <br> `vtheta`: Rotation speed (rad/s) |
| `agent.stop()` | Stop all movement. |
| `agent.kick()` | Perform a kick action. |
| `agent.head_control(pitch, yaw)` | Control head/neck angles. |

## 5. Development Workflow

1.  **Launch Simulation**: `launch_sim.sh`
2.  **Edit Strategy**: Modify `decider/user_entry.py`.
3.  **Test Single Robot**: Run `decider.py` manually to debug.
    ```bash
    python3 decider/decider.py --simulation --color red --id 0
    ```
4.  **Run Full Match**: Use `start_team.sh` to see multi-agent behavior.
