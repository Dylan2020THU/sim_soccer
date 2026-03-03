# MOS-Brain Documentation

[中文文档 / Chinese Version](./README_zh.md)

This repository mainly includes:
- `decider`: decision logic and strategy development
- `mujoco`: Mujoco simulation environment migrated from Isaac Sim, plus simulation management tools
- `simulation`: [Isaac Sim simulation environment](./simulation/README.md)

Decider module notes: [decider/README.md](./decider/README.md)

## License

- Main project license: **GPLv3** (`./LICENSE`)
- Third-party attribution and license notes: `./THIRD_PARTY_NOTICES.md`

## 1. Quick Start

### Step 1: SSH Login to Server
Before running simulation and experiments, connect to the designated server.
1. **Request an account**: Contact **Luo Shaoyin**.
2. **Confirm NAT mapping**: IP `166.111.192.4`, Port `55222`.
3. **Connect**:
   ```bash
   ssh -p 55222 your_username@166.111.192.4
   ```

### Step 2: Sim Manager (Mujoco Simulation)

Use an already deployed Sim Manager service.

Pages:
- Manager UI: `http://127.0.0.1:8000/`
- API Docs page: `http://127.0.0.1:8000/manager/docs`
- Swagger: `http://127.0.0.1:8000/docs`

Detailed startup parameters for server deployers (simulation service maintainers):
- [Mujoco README: Sim Manager (FastAPI)](./mujoco/README.md#sim-manager-使用说明推荐先看)

#### Step 2.1: Sim Manager Usage Details

1. **Start simulation instances**
- In `Start Simulation`, set `team_size` and optional rendering parameters.
- `zmq_port` and `webview_port` can be left empty; free ports will be assigned automatically.
- Click `Start`.

2. **Check allocated resources**
- In `Scanned Sim Processes`, check:
  - `PID`
  - `ZMQ Port` (used by decider control)
  - `Team Size`
  - `WebView` link

3. **Stop simulation processes**
- In `Process Control`, choose PID from the dropdown, then click `Stop PID`.
- Click `Stop External` to stop scanned simulation processes not created by this manager.

4. **Generate decider commands**
- Select a PID and expand `Decider Commands`.
- Commands are generated for both teams and each robot index.
- Copy and run them from the `mos-brain` root directory.

5. **Check logs**
- `Start Log`: start results
- `Action Log`: stop/cleanup results

### Step 3: Decider Environment Setup and Start

Tip: Exact decider startup commands can be copied directly from Sim Manager (`Process Control` -> `Decider Commands`).

Set up decider environment:

```bash
cd ./mos-brain
conda create -n k1 python=3.8 -y
conda activate k1
pip install -r decider/requirements.txt
```

Start one robot in simulation mode (replace `--port` with the ZMQ port assigned by Sim Manager):

```bash
# Red team player 0
python3 decider/decider.py --simulation --ip 127.0.0.1 --port 5555 --color red --id 0

# Blue team player 0
python3 decider/decider.py --simulation --ip 127.0.0.1 --port 5555 --color blue --id 0
```

Note: Blue team IDs are automatically offset according to match config.

#### Option B: Start full teams

```bash
./decider/scripts/start_team.sh
```

Useful commands:
- Attach one robot log: `screen -r decider_red_0` (detach with `Ctrl+A`, `D`)
- Restart all robots: `./decider/scripts/start_team.sh --restart`
- Stop all robots: `./decider/scripts/start_team.sh --kill`
- Override team sizes: `./decider/scripts/start_team.sh --red 2 --blue 1`

If leftover decider processes exist:
```bash
ps -ef | grep decider.py | grep -v grep
sudo pkill -f decider.py
```

## 2. Decider Configuration

Edit `decider/config.yaml` to adjust defaults:
- `color`: default team color
- `id`: default robot ID

### `cmd_vel` clipping

`decider` no longer applies dedicated `cmd_vel` clipping in `config.yaml`.
Command shaping/scaling is still handled by the control logic itself.

## 3. Writing Custom Strategy

Main entry: `decider/user_entry.py`

### `game(agent)` Loop

```python
def game(agent):
    if not agent.get_if_ball():
        agent.state_machine_runners['find_ball']()
    else:
        agent.state_machine_runners['chase_ball']()
```

### Built-in State Machines

Located in `decider/logic/sub_statemachines/`:
- `find_ball`
- `chase_ball`
- `dribble`
- `kick`
- `go_back_to_field`

Usage:
```python
agent.state_machine_runners['state_machine_name']()
```

## 4. Decider API Reference

Interface files:
- Action: `decider/interfaces/action.py`
- Vision: `decider/interfaces/vision.py`
- GameController: `decider/interfaces/gamecontroller.py`

### Coordinate Systems

- **Robot Frame**:
  - `X`: forward (meters)
  - `Y`: left (meters)
  - `Theta`: counter-clockwise (radians), `0` means forward
- **Map Frame**:
  - `Y`: toward opponent goal
  - `X`: to the right
  - Blue team is mirrored relative to simulator global coordinates so both teams can reuse the same strategy logic.

### Perception API

| Method | Description | Return |
|--------|-------------|--------|
| `agent.get_ball_pos()` | Ball position in Robot Frame | `[x, y]` or `[None, None]` |
| `agent.get_ball_distance()` | Distance to ball | `float` |
| `agent.get_ball_angle()` | Ball angle in Robot Frame | `float` |
| `agent.get_if_ball()` | Whether ball is visible | `bool` |
| `agent.get_self_pos()` | Robot position in Map Frame | `[x, y]` |
| `agent.get_self_yaw()` | Robot heading in Map Frame | `float` |

### Action API

| Method | Description |
|--------|-------------|
| `agent.cmd_vel(vx, vy, vtheta)` | Velocity control in Robot Frame |
| `agent.stop()` | Stop movement |
| `agent.kick()` | Execute kick |
| `agent.head_control(pitch, yaw)` | Head angle control |
