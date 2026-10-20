(bridge-service-overview)=
# 服务概述

**权威列表：** `automsgs/proto/rpcs/*.proto`（文档可用 `check_bridge_rpc_docs.py` 校验）。
接入步骤：[01](01_connection_guide.md)。

**什么时候看：** 选 proto、写客户端前，确认服务名与 RPC 模式。

### 域服务

| 服务 | Proto | 主要 RPC | 分类 |
|------|-------|----------|------|
| `NavigationService` | `navigation.proto` | Navigate · Pause/Resume/Replan/Cancel · GetStatus | Command |
| `FollowService` | `follow.proto` | Follow · 生命周期 | Command |
| `TeleopService` | `teleop.proto` | Velocity(Bidi) · Drive/BackUp/Spin | Command |
| `ChargeService` | `charge.proto` | Return · Leave | Command |
| `MapService` | `mapping.proto` | 建图会话 · List/Get/Save Map | Map |
| `ExplorationService` | `exploration.proto` | Explore · SetArea · SaveMap | Command |
| `VoiceService` | `voice.proto` | Execute · Cancel | Command |
| `LocalizationService` | `localization.proto` | GetPose · SetInitialPose | Query |
| `SensorService` | `sensor.proto` | ListSensors · GetSample · Record | Query/Stream |
| `SystemService` | `system.proto` | Get* · Estop · CancelAll · Caps | Query/System |

### 模式与互斥

| 模式 | 示例 |
|------|------|
| Unary | System、Loc、Map 资源、生命周期 Pause |
| Unary → Stream | Navigate、Follow、Return、Explore |
| Bidi | `TeleopService/Velocity` |

- Command 带 `goal_id`；**TaskMuxer** 保证同一时刻通常只有一个互斥 Command 占槽。  
- `EmergencyStop` / `CancelAllGoals` 可打断 Stream。  
- busy 时各域 Status code 不同（如 `NAVIGATION_BUSY=105`），见 `status_msgs` 与 proto 注释。

### TaskType（与 GetActiveGoal 对齐）

| 值 | 类型 | 典型服务 |
|----|------|----------|
| 1 | NAVIGATION | Navigate |
| 2 | FOLLOW | Follow |
| 3 | TELEOP | Teleop |
| 4 | EXPLORATION | Explore |
| 5 | DOCK/CHARGE | Charge |
| 6 | MAP | Map 会话 |
| 8 | VOICE | Voice |

Handler 接线进度：[grpc/07](../grpc/07_handlers.md)。
