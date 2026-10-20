(bridge-service-overview)=
# 服务概述

对外表面：`automsgs.rpcs.*`（`automsgs/proto/rpcs/`）。接入：[01](01_connection_guide.md)。

### 2.1 域服务

| 服务 | Proto | 主要 RPC | 分类 |
|------|-------|----------|------|
| `NavigationService` | `navigation.proto` | Navigate · Pause/Resume/Replan/Cancel · GetStatus | Command |
| `FollowService` | `follow.proto` | Follow · 生命周期 | Command |
| `TeleopService` | `teleop.proto` | Velocity · Drive/BackUp/Spin | Command |
| `ChargeService` | `charge.proto` | Return · Leave | Command |
| `MapService` | `mapping.proto` | Mapping · List/Get/Save Map | Command/Query |
| `ExplorationService` | `exploration.proto` | Explore · SetArea · SaveMap | Command |
| `VoiceService` | `voice.proto` | Execute · Cancel | Command |
| `LocalizationService` | `localization.proto` | GetPose · SetInitialPose | Query |
| `SensorService` | `sensor.proto` | ListSensors · GetSample · Record | Query/Stream |
| `SystemService` | `system.proto` | Get* · Estop · CancelAll · Caps | Query/System |

完整列表以 proto 为准。

### 2.2 模式与约束

| 模式 | 示例 |
|------|------|
| Unary | System / Loc / Map 资源 |
| Unary→Stream | Navigate / Follow / Return |
| Bidi | `TeleopService/Velocity` |

- 命令带 `goal_id`；同一时刻仅一个互斥 Command（`TaskMuxer`）
- Estop / CancelAll 可打断

### 2.3 TaskType

| 值 | 类型 | 服务 |
|----|------|------|
| 1 | NAVIGATION | Navigate |
| 2 | FOLLOW | Follow |
| 3 | TELEOP | Teleop |
| 4 | EXPLORATION | Explore |
| 5 | DOCK | Charge Return/Leave |
| 6 | MAP | MapService |
| 8 | VOICE | Voice Execute |
