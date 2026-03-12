# User Entry Algorithm (`decider/user_entry.py`)

本文档说明当前 `user_entry.py` 的状态调度与入口模式。

## 1. 当前行为变更

- `dribble` 与 `adv_dribble` 均使用原始 `AdvancedDribbler.run`（Adv 算法）。
- 不再引用 `DribbleStateMachine` 作为运行入口。
- `dribble` runner 对外保持兼容：即使传入 `aim_yaw` 等参数也会被安全忽略。
- `adv_dribble_legacy` 仍保留为同一 Adv 实现的兼容别名。

## 2. 初始化的 runner

`init()` 中注册如下 runner：

- `find_ball` -> `FindBallStateMachine.run`
- `chase_ball` -> `ChaseBallStateMachine.run`
- `go_back_to_field` -> `GoBackToFieldStateMachine.run`
- `dribble` -> `AdvancedDribbler.run`（兼容包装）
- `adv_dribble` -> `AdvancedDribbler.run`（别名）
- `adv_dribble_legacy` -> `AdvancedDribbler.run`
- `goalkeeper` -> `GoalkeeperStateMachine.run`
- `stop` -> `agent.stop`

## 3. `game()` 模式分发

`game()` 按以下优先级选择模式：

1. `agent.get_command()["data"]["test_mode"]`
2. `config.yaml` 中 `user_entry.mode`
3. 默认值：`playing`

支持模式：

- `playing`：完整流程（看不到球 `find_ball`，远球 `chase_ball`，近球 `adv_dribble`）
- `adv_dribble`：测试 `adv_dribble`（当前为 Adv 算法）
- `adv_dribble_legacy`：测试旧 `AdvancedDribbler`
- `dribble`：测试 `dribble`（当前同样为 Adv 算法，兼容 `aim_yaw` 参数）
- `find_ball`：仅找球
- `chase_ball`：追球（无球时回退到找球）
- `go_back_to_field`：回场（从 command data 读取 `aim_x/aim_y/aim_yaw`）
- `goalkeeper`：门将策略
- `stop`：停止

未知模式会告警并回退到 `playing`。
