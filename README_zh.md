# MOS-Brain 文档（中文）

> AI 翻译声明：本中文文档由 AI 基于英文文档生成，更新可能存在滞后；如有冲突，请以英文版为准。

[English Version](./README.md)

本仓库主要包含：
- `decider`：决策逻辑与策略开发
- `simulation/mujoco`：迁移自 Isaac Sim 的 Mujoco 仿真环境和仿真环境管理工具
- `simulation/isaac_sim`：[Isaac Sim 仿真环境](./simulation/isaac_sim/README.md)
- `simulation/labbridge`：独立的 WebView/bridge/sim-manager 模块

Decider 模块说明：[`decider/README.md`](./decider/README.md)

## 许可证

- 项目主许可证：**GPLv3**（`./LICENSE`）
- SPDX 许可证标识：`GPL-3.0-or-later`
- 第三方来源与许可证说明：`./THIRD_PARTY_NOTICES.md`

## 1. 快速开始

### Step 1: SSH 登录服务器
运行仿真和实验前，请先连接指定服务器。
1. **申请账号**：联系 **Luo Shaoyin** 开通账号。
2. **确认 NAT**：确认映射信息（IP: `166.111.192.4`, Port: `55422`）。
3. **连接命令**：
   ```bash
   ssh -p 55422 your_username@166.111.192.4
   ```
4. **注意**：我们使用了不同的机器并且更改了端口号，您在新机器的账户可能需要重新申请。

### Step 2: Sim Manager（Mujoco 仿真）

使用已部署好的 Sim Manager 服务。

页面入口：
- Manager 页面：`http://127.0.0.1:8000/`
- API 说明页：`http://127.0.0.1:8000/manager/docs`
- Swagger：`http://127.0.0.1:8000/docs`

给服务器部署者（仿真服务维护者）看的详细启动参数：
- [Mujoco README: Sim Manager (FastAPI)](./simulation/mujoco/README.md#sim-manager-使用说明推荐先看)

#### Step 2.1: Sim Manager 详细使用

1. **启动仿真实例**
- 在 `Start Simulation` 里设置 `team_size` 和可选渲染参数。
- `zmq_port`、`webview_port` 可留空，系统会自动分配可用端口。
- 点击 `Start`。

2. **查看实例资源**
- 在 `Scanned Sim Processes` 查看：
  - `PID`
  - `ZMQ Port`（给 decider 控制使用）
  - `Team Size`
  - `WebView` 链接

3. **停止仿真进程**
- 在 `Process Control` 下拉选择 PID，点击 `Stop PID`。
- 点击 `Stop External` 可停止非本 manager 创建但被扫描到的仿真进程。

4. **生成 decider 命令**
- 选择 PID 后展开 `Decider Commands`。
- 页面会按红蓝队、按机器人编号生成命令。
- 点击复制后在 `mos-brain` 根目录执行。

5. **查看日志**
- `Start Log`：启动结果
- `Action Log`：停止/清理结果

### Step 3: 决策环境安装与启动

提示：具体的 decider 启动命令可以直接从 Sim Manager 页面复制（`Process Control` -> `Decider Commands`）。

安装决策环境：

```bash
cd ./mos-brain
conda create -n k1 python=3.8 -y
conda activate k1
pip install -r decider/requirements.txt
```

以仿真模式启动单机器人（`--port` 换成 Sim Manager 分配的 ZMQ 端口）：

```bash
# 红队 0 号
python3 decider/decider.py --simulation --ip 127.0.0.1 --port 5555 --color red --id 0

# 蓝队 0 号
python3 decider/decider.py --simulation --ip 127.0.0.1 --port 5555 --color blue --id 0
```

说明：蓝队会根据 match config 自动做队伍 ID 偏移。

#### 方案 B：启动整队

```bash
./decider/scripts/start_team.sh
```

常用命令：
- 查看单个机器人日志：`screen -r decider_red_0`（`Ctrl+A`, `D` 退出）
- 重启全部机器人：`./decider/scripts/start_team.sh --restart`
- 停止全部机器人：`./decider/scripts/start_team.sh --kill`
- 指定数量启动：`./decider/scripts/start_team.sh --red 2 --blue 1`

若有残留 decider 进程：
```bash
ps -ef | grep decider.py | grep -v grep
sudo pkill -f decider.py
```

## 2. Decider 配置

编辑 `decider/config.yaml` 调整默认参数：
- `color`：默认队伍颜色
- `id`：默认机器人 ID

### `cmd_vel` 处理

Decider 目前不再在 `config.yaml` 中提供专门的 `cmd_vel` 限幅项。
速度命令仍由控制逻辑本身完成缩放与整形。

## 3. 编写自定义策略

主入口：`decider/user_entry.py`

### `game(agent)` 主循环

```python
def game(agent):
    if not agent.get_if_ball():
        agent.state_machine_runners['find_ball']()
    else:
        agent.state_machine_runners['chase_ball']()
```

### 内置状态机

位于 `decider/logic/sub_statemachines/`：
- `find_ball`
- `chase_ball`
- `dribble`
- `kick`
- `go_back_to_field`

使用方式：
```python
agent.state_machine_runners['state_machine_name']()
```

## 4. Decider API 参考

接口文件：
- Action：`decider/interfaces/action.py`
- Vision：`decider/interfaces/vision.py`
- GameController：`decider/interfaces/gamecontroller.py`

### 坐标系

- **机器人坐标系（Robot Frame）**：
  - `X`：前向（米）
  - `Y`：左向（米）
  - `Theta`：逆时针（弧度），`0` 为正前方
- **地图坐标系（Map Frame）**：
  - `Y`：朝向对方球门
  - `X`：右侧
  - 蓝队相对仿真全局坐标做了镜像处理，便于两队复用同一套策略。

### 感知 API

| 方法 | 说明 | 返回 |
|--------|-------------|---------|
| `agent.get_ball_pos()` | 小球在机器人坐标系的位置 | `[x, y]` 或 `[None, None]` |
| `agent.get_ball_distance()` | 到球距离 | `float` |
| `agent.get_ball_angle()` | 小球角度（机器人坐标系） | `float` |
| `agent.get_if_ball()` | 是否看见球 | `bool` |
| `agent.get_self_pos()` | 机器人地图坐标 | `[x, y]` |
| `agent.get_self_yaw()` | 机器人地图朝向 | `float` |

### 动作 API

| 方法 | 说明 |
|--------|-------------|
| `agent.cmd_vel(vx, vy, vtheta)` | 机器人坐标系速度控制 |
| `agent.stop()` | 停止运动 |
| `agent.kick()` | 执行踢球 |
| `agent.head_control(pitch, yaw)` | 头部角度控制 |
