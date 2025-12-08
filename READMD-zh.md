# Agent 实现快速指南

> Translated by Gemini 
> May not be up-to-date

为了帮助您快速实现自己的策略或算法，此框架将所有复杂的、与平台相关的接口封装在 `Agent` 类中。

您的决策代码应以类似于 Arduino IDE 的结构开发，需要在 `user_entry.py` 文件中提供以下两个函数：

* **`def init(agent) -> None`**：此函数将在系统启动期间调用一次，即在所有初始组件（ROS 2 节点、接口）初始化完成之后、主循环开始之前。用于一次性设置和初始化工作。

* **`def loop(agent) -> None`**：此函数将以固定频率（通常为 10Hz，或由 `timer_period` 定义）持续调用。这是您的策略主控制循环。此函数在任何情况下**不得**阻塞或等待。

传递给这些函数的 `agent` 实例是与环境交互的唯一接口，封装了传感器采集、自身定位和运动控制的逻辑。

## Agent 接口

`Agent` 类提供了一套全面的方法，分为执行（输出）和感知（输入）两类。

### 执行 (Actuations)

1.  **`cmd_vel(self, vel_x: float, vel_y: float, vel_theta: float) -> None`**
    设置机器人的目标**相对**速度。这些值会根据配置限制（例如 `max_walk_vel_x`）进行缩放和保护。

    * `vel_x`: 目标速度 (m/s)，指向正前方（正值）或后方（负值）。

    * `vel_y`: 目标速度 (m/s)，指向左侧（正值）或右侧（负值）。

    * `vel_theta`: 目标角速度 (rad/s)，用于逆时针旋转（正值）或顺时针旋转（负值）。

2.  **`move_head(self, pitch: float, yaw: float) -> None`**
    设置机器人的头部和颈部角度，允许用户暂时覆盖自主头部控制组件。

    * `pitch`: 头部俯仰（上下倾斜）的角度。

    * `yaw`: 头部偏航（左右摇晃）的角度。

    **注意：** 调用此方法接管头部控制。将参数设置为 `math.nan` 以将控制权返回给专用的 `head_control` 组件。

3.  **`kick(self, foot=0, death=0) -> None`**
    执行踢球动作。

    * `foot`: `0` 表示左脚，`1` 表示右脚。

    * `death`: 设置为 `1` 以执行可能导致机器人摔倒的强力踢球。**谨慎使用。**

4.  **`save_ball(self, direction=1) -> None`**
    执行守门扑救动作。

    * `direction`: `1` 表示向左扑救，`2` 表示向右扑救。

    **谨慎使用。**

5.  **`stop(self, sleep_time: float = 0) -> None`**
    立即停止所有机器人运动。

    * `sleep_time`: 发布停止命令后可选的延迟时间（秒），以确保进入下一个命令前的稳定。

### 感知 (Perception)

#### 自身定位与配置

1.  **`get_self_pos(self) -> np.ndarray`**
    返回机器人在全局地图坐标中的当前位置 (x, y)。

    * **返回类型:** `np.ndarray` (`[x, y]`)

2.  **`get_self_yaw(self) -> float`**
    返回机器人在地图坐标中的当前朝向（偏航角，弧度）。

    * **返回类型:** `float`

3.  **`angle_normalize(self, angle: float) -> float`**
    一个工具函数，用于将任意给定角度（弧度）归一化到标准范围 $(-\pi, \pi)$ 内。如果输入为 `None` 则返回 `None`。

    * **返回类型:** `Optional[float]`

4.  **`get_config(self) -> Dict`**
    返回 Agent 的完整配置字典，通常从 YAML 或 JSON 文件加载。

    * **返回类型:** `Dict`

#### 球和视觉数据

1.  **`get_if_ball(self) -> bool`**
    如果视觉系统当前检测到球，则返回 `True`，否则返回 `False`。

    * **返回类型:** `bool`

2.  **`get_ball_pos(self) -> List[Optional[float]]`**
    返回球相对于机器人当前位置的坐标（`[x, y]`，单位米），如果未检测到球，则返回 `[None, None]`。

    * **返回类型:** `List[Optional[float]]` (`[x, y]`)

3.  **`get_ball_distance(self) -> float`**
    返回到球的距离（米）。如果未检测到球，则返回一个较大的值（`1e6`）。

    * **返回类型:** `float`

4.  **`get_ball_angle(self) -> Optional[float]`**
    计算并返回球相对于机器人前进方向的角度（弧度）。如果未检测到球，则返回 `None`。

    * **返回类型:** `Optional[float]`

5.  **`get_ball_pos_in_map(self) -> np.ndarray`**
    返回球在全局地图坐标中的位置 (x, y)。如果未检测到球，则返回 `None`。

    * **返回类型:** `np.ndarray` (`[x, y]`)

6.  **`get_if_close_to_ball(self) -> bool`**
    如果到球的距离小于配置的 `close_to_ball_threshold`，则返回 `True`。

    * **返回类型:** `bool`

7.  **`get_ball_history(self)`**
    返回球的检测位置及其关联元数据的最近历史记录。具体的返回类型由底层视觉接口定义。

    * **返回类型:** `Any`

8.  **`get_neck(self) -> float`**
    返回机器人颈部关节的当前角度（弧度）。

    * **返回类型:** `float`

9.  **`get_head(self) -> float`**
    返回机器人头部关节的当前角度（弧度）。

    * **返回类型:** `float`

# 代码结构

### 强制结构 (Mandatory structures)
由于 systemd 服务的自动启动，存在两个强制目录：
* `decider/scripts/*`

**您绝不应该更改此目录下的任何文件的权限、删除、重命名或修改其文件结构。**

以及 Agent 的依赖项：
* `interfaces/*`：包含底层接口的兼容层。
* `configuration.py`：包含用于解析 `config.yaml` 的代码。
* `config.yaml`：是配置文件。不要删除或修改，但可以自由追加内容。
* `config_override.yaml`：是一个 Git 不会跟踪的文件。如果您的配置是机器人特有的且不应被 Git 跟踪，请将您的配置放在这里。创建或不创建，此文件都是可选的。
* `user_entry.py`：是包含 `loop(agent)` 和 `init(agent)` 的入口文件。将您的代码放在此处。

### 日志记录 (Logging)
我们使用与 Python 标准库 `logging` 兼容的日志记录函数。示例：
```python
logger = agent.get_logger()
logger.debug(f"...") # 调试级别，默认隐藏
logger.info(f"...")  # 信息级别
logger.warning(f"...") # 警告级别
logger.error(f"...") # 错误级别
```