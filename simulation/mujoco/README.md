# K1 MuJoCo Multi-Robot

## Sim Manager 使用说明（推荐先看）

Sim Manager 可以用来管理所有 MuJoCo 仿真环境，替代命令行启动 MuJoCo 仿真。

说明：Sim Manager 主实现已迁移到独立模块 `simulation/labbridge/sim_manager.py`。  
`simulation/mujoco/sim_manager.py` 目前是兼容入口，会转发到 `simulation.labbridge` 实现。

1. 启动 manager：

```bash
cd ./mos-brain
conda run -n mujoco312 uvicorn simulation.labbridge.sim_manager:app --host 0.0.0.0 --port 8000
```

兼容旧方式（仍可用）：

```bash
cd ./mos-brain/simulation/mujoco
conda run -n mujoco312 python sim_manager.py --host 0.0.0.0 --port 8000
```

2. 打开页面：
- Manager 前端：`http://127.0.0.1:8000/`
- API 说明页：`http://127.0.0.1:8000/manager/docs`
- Swagger：`http://127.0.0.1:8000/docs`

3. 在 `Start Simulation` 设置参数后点击 `Start`：
- `zmq_port`、`webview_port` 可留空，manager 会自动分配可用端口
- `policy_device` 默认 `gpu`；若无 CUDA，会自动回退 `cpu`

4. 在 `Scanned Sim Processes` 查看：
- PID、ZMQ Port、Team Size、WebView 链接

5. 在 `Process Control`：
- 通过下拉选择扫描到的 PID，执行 `Stop PID`
- 执行 `Stop External` 停止非 manager 创建但被扫描到的仿真

6. 选择 PID 后展开 `Decider Commands`：
- 可直接复制每个机器人的 decider 启动命令

7. 关闭 manager 时：
- 会自动停止该 manager 创建的全部仿真进程

## 将 Sim Manager 持久化为 systemd 服务

适用于需要断开 SSH 后持续运行，或开机自动拉起 manager 的场景。
部署服务时建议使用 `uvicorn`（或 `python -m uvicorn`）作为 ASGI 服务器；
`python sim_manager.py` 仅保留为兼容入口/调试入口。

在 `mos-brain` 目录执行：

```bash
PROJECT_ROOT="$(pwd)"
PYTHON_BIN="$(conda run -n mujoco312 which python)"

sudo tee /etc/systemd/system/mujoco-sim-manager.service > /dev/null <<EOF
[Unit]
Description=MuJoCo Sim Manager
After=network.target

[Service]
Type=simple
WorkingDirectory=${PROJECT_ROOT}
ExecStart=${PYTHON_BIN} -m uvicorn simulation.labbridge.sim_manager:app --host 0.0.0.0 --port 8000
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
```

启用并启动：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now mujoco-sim-manager
```

常用管理命令：

```bash
systemctl status mujoco-sim-manager
journalctl -u mujoco-sim-manager -f
sudo systemctl restart mujoco-sim-manager
sudo systemctl stop mujoco-sim-manager
```

服务停止后不会自动清理历史仿真进程；如需清理，可在 Manager 页面执行 `Stop External`。

### 最小权限运行（推荐）

上面的命令使用 `sudo` 仅用于注册 systemd 服务；`sim_manager.py` 进程本身建议以普通用户运行。

示例（system 级服务，进程降权）：

```bash
PROJECT_ROOT="$(pwd)"
PYTHON_BIN="$(conda run -n mujoco312 which python)"

sudo tee /etc/systemd/system/mujoco-sim-manager.service > /dev/null <<EOF
[Unit]
Description=MuJoCo Sim Manager
After=network.target

[Service]
Type=simple
User=simrunner
Group=simrunner
WorkingDirectory=${PROJECT_ROOT}
ExecStart=${PYTHON_BIN} -m uvicorn simulation.labbridge.sim_manager:app --host 0.0.0.0 --port 8000
Restart=always
RestartSec=3
NoNewPrivileges=true
PrivateTmp=true
ProtectSystem=strict
ProtectHome=true
ReadWritePaths=${PROJECT_ROOT}

[Install]
WantedBy=multi-user.target
EOF
```

如果只需要为当前用户运行，也可使用 user 级服务（不写 `/etc/systemd/system`，无需 sudo）：

```bash
mkdir -p ~/.config/systemd/user
PROJECT_ROOT="$(pwd)"
PYTHON_BIN="$(conda run -n mujoco312 which python)"

tee ~/.config/systemd/user/mujoco-sim-manager.service > /dev/null <<EOF
[Unit]
Description=MuJoCo Sim Manager (User)
After=default.target

[Service]
Type=simple
WorkingDirectory=${PROJECT_ROOT}
ExecStart=${PYTHON_BIN} -m uvicorn simulation.labbridge.sim_manager:app --host 0.0.0.0 --port 8000
Restart=always
RestartSec=3

[Install]
WantedBy=default.target
EOF

systemctl --user daemon-reload
systemctl --user enable --now mujoco-sim-manager
```

需要“退出登录后继续运行”时，管理员可执行：

```bash
sudo loginctl enable-linger <username>
```

入口脚本：`k1_sim2sim_runner.py`  
核心代码：`app/multi_robot_sim.py`  
启动入口：`app/runner.py`

## 环境安装

```bash
cd ./mos-brain/simulation/mujoco
conda create -n mujoco312 python=3.12 -y
conda activate mujoco312
pip install -r requirements.txt
```

快速检查依赖是否就绪：

```bash
python -c "import mujoco, torch, zmq, flask, fastapi, uvicorn; print('ok')"
```

## 运行

```bash
cd ./mos-brain/simulation/mujoco
conda run -n mujoco312 python k1_sim2sim_runner.py --team-size 3
```

- `--robot-type`：机器人类型，可选 `k1`（默认）或 `pi_plus`
  - `k1` 默认策略：`assets/policies/k1_model_46000.pt`
  - `pi_plus` 默认策略：`assets/policies/pi_plus_model_40000.pt`
- `--team-size`：每队机器人数量（红蓝相等），范围 `0..7`，默认 `1`
- `--use-referee`：启用内置裁判盒（开球/出界/角球/门球/进球/双触球/超时）
  - 定位球阶段仅允许对应球队命令生效（另一方命令会被忽略）
- WebView 默认 `http://localhost:5811`
- ZMQ REP 默认 `tcp://*:5555`

示例（启动 `pi_plus`）：

```bash
conda run -n mujoco312 python k1_sim2sim_runner.py --robot-type pi_plus --team-size 3
```

## cmd_vel 输入处理

当前 `mujoco` 不再对外部输入的 `vx/vy/ang_z` 做统一限幅。
Web 输入与 ZMQ 输入会按原值写入命令缓冲。

## 固定 Robot ID 映射（始终不变）

- `0..6 -> robot_rp0..robot_rp6`（红队）
- `7..13 -> robot_bp0..robot_bp6`（蓝队）

说明：即使本次 `--team-size` 小于 7，映射仍固定；未启用 ID 会被忽略。

## ZMQ 协议（对齐 simulation/sim_server.py）

请求（单指令）：

```json
{"cmd":[vx,vy,w], "id":0, "timestamp": 0, "source":"xxx"}
```

请求（批量兼容）：

```json
{"commands":[{"cmd":[vx,vy,w], "id":0, "timestamp":0, "source":"xxx"}]}
```

响应：

```json
{
  "state": {
    "robots": [{"id":0, "name":"robot_rp0", "x":0, "y":0, "theta":0, "team":"red"}],
    "ball": {"x":0, "y":0, "z":0}
  },
  "sim_timestamp": 0,
  "step_latency": 0,
  "ack_timestamp": 0
}
```

## Reset/拖动语义

- Web `reset`：机器人和球都回到初始位置（全量复位），但不重置 referee 比赛状态
- Web `restart match`：重置 referee，并将机器人和球一起重新生成到初始位置
- 小地图拖动机器人：reset 后只改变该机器人位置，球保持不变
- 小地图拖动球：直接改球位置，不触发 reset，机器人状态保持不变

## Sim Manager (FastAPI)

用于统一管理仿真进程（启动/扫描/停止），包括停止不是 manager 创建但被扫描到的仿真。

启动 manager：

```bash
cd ./mos-brain
conda run -n mujoco312 uvicorn simulation.labbridge.sim_manager:app --host 0.0.0.0 --port 8000
```

页面入口：

- Manager 前端：`http://127.0.0.1:8000/`
- API 说明页：`http://127.0.0.1:8000/manager/docs`
- Swagger：`http://127.0.0.1:8000/docs`

说明：Manager 前端的 `Scanned Sim Processes` 表会显示可点击的 WebView 链接（如果该进程开启了 webview）。

接口：

- `GET /healthz`：健康检查
- `POST /sims/start`：启动一个仿真进程
- `GET /sims`：扫描当前仿真进程，返回 manager 创建和外部进程
- `POST /sims/stop`：按 `pid` 停止扫描到的仿真（不要求是 manager 创建）
- `POST /sims/stop-external`：批量停止扫描到的“外部仿真进程”（非 manager 创建）

`POST /sims/start` 请求示例：

```json
{
  "team_size": 3,
  "zmq_port": null,
  "webview_port": null,
  "webview": true,
  "zmq": true,
  "use_referee": false
}
```

说明：`zmq_port` 和 `webview_port` 可留空（或 `null`），manager 会自动选择可用端口并在返回中给出 `resolved.zmq_port` / `resolved.webview_port`。
说明：`use_referee=true` 时，仿真启用内置裁判盒并在状态中附带 `gamecontroller`（play mode / score / play time）。

`POST /sims/stop` 请求示例：

```json
{
  "pid": 12345
}
```
