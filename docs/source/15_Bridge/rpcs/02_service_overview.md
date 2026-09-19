(bridge-service-overview)=
# automsgs.rpcs 服务概述

Bridge 对外暴露的 gRPC 表面为 **`automsgs.rpcs.*`**（定义于 `automsgs/proto/rpcs/`）。已移除历史 `AutonomyService` / `external_command_service.proto`。

接入步骤见 [01 接入与验证](01_connection_guide.md)。第三方优先用 `automsgs/tools/cli/rpc-cli.py`。

## 2.1 域服务与方法

| 服务 | Proto | 主要 RPC | 分类 |
|------|-------|----------|------|
| `NavigationService` | `navigation.proto` | `Navigate` · `Pause` · `Resume` · `Replan` · `Cancel` · `GetStatus` | Command |
| `FollowService` | `follow.proto` | `Follow` · `Pause` · `Resume` · `Cancel` · `GetStatus` | Command |
| `TeleopService` | `teleop.proto` | `Velocity` · `DriveOnHeading` · `BackUp` · `Spin` · 生命周期 | Command |
| `ChargeService` | `charge.proto` | `Return` · `Leave` · 生命周期 | Command |
| `MapService` | `mapping.proto` | `StartMapping` · `FinishMapping` · `ListMaps` · `GetMap` … | Command / Query |
| `ExplorationService` | `exploration.proto` | `Explore` · `SetArea` · `SaveMap` · 生命周期 | Command |
| `VoiceService` | `voice.proto` | `Execute` · `Cancel` · `GetStatus` | Command |
| `LocalizationService` | `localization.proto` | `GetPose` · `GetStatus` · `SetInitialPose` | Query |
| `SensorService` | `sensor.proto` | `ListSensors` · `GetSample` · `Record` … | Query / Stream |
| `SystemService` | `system.proto` | `Heartbeat` · `GetInfo` · `GetHealth` · `GetRobotFullInfo` · `EmergencyStop` · `CancelAllGoals` · `GetActiveGoal` · `GetCapabilities` | Query / System |

完整 `rpc` 列表以 `automsgs/proto/rpcs/*.proto` 为准；契约校验：`python3 docs/scripts/check_bridge_rpc_docs.py`。

## 2.2 RPC 分类

| 分类 | 用途 | 测试关注点 |
|------|------|------------|
| **Query** | 无副作用只读查询 | 连通性、能力矩阵、版本号 |
| **Stream** | 长连接命令进度 / 录制 | 频率、末帧、断线重连 |
| **System** | 全局急停 / 取消 | 抢占行为、幂等 |
| **Command** | 下发机载任务 | Stream 进度、`TaskType` 互斥 |

## 2.3 流模式

| 模式 | 方法示例 | 客户端 |
|------|----------|--------|
| Unary | System / Loc / Map 资源 | 单次 `Invoke` |
| Unary → Stream | `Navigate` · `Follow` · `Return` … | 发一帧 Request，读 Stream 至终态 |
| Bidi Stream | `TeleopService/Velocity` | 双工读写 |

## 2.4 调用约束

- 命令请求带 `goal_id`（或等价字段）；System / 公共类型见 [03](03_common_types.md) 与 `automsgs/rpcs/common.proto`。
- **同一时刻仅一个互斥 Command 任务**（`TaskMuxer`）；`EmergencyStop` / `CancelAllGoals` 可打断。
- MQTT Topic 与字段同名，见 [mqtt/04 §4.1](../mqtt/04_topic_protocol.md#41-topic-命名)（若仍启用）。

## 2.5 Command 与 TaskType 对照

| TaskType | 值 | 服务 / RPC | `supports_*` 门控 |
|----------|-----|------------|-------------------|
| `TASK_TYPE_NAVIGATION` | 1 | `NavigationService/Navigate` | `supports_navigation` |
| `TASK_TYPE_FOLLOW` | 2 | `FollowService/Follow` | `supports_follow` |
| `TASK_TYPE_TELEOP` | 3 | `TeleopService/*` | `supports_teleop` |
| `TASK_TYPE_EXPLORATION` | 4 | `ExplorationService/Explore` | `supports_exploration` |
| `TASK_TYPE_DOCK` | 5 | `ChargeService/Return|Leave` | `supports_docking` |
| `TASK_TYPE_MAP` | 6 | `MapService` 建图命令 | `supports_map_management` |
| `TASK_TYPE_VOICE` | 8 | `VoiceService/Execute` | — |

语音控制走 `VoiceService/Execute` → Bridge `VoiceStub` GoalChannel → `VoiceTask`，由 task 内再分发到导航 / 跟随 / 回充 / 探索 / 取消。探索 / 导航同理：`ExplorationStub` / `NavigatorStub` 仅触发对应 Task，编排在 `autonomy/task`。
