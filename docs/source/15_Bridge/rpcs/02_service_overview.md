(bridge-service-overview)=
# AutonomyService 服务概述

Bridge 对外暴露的唯一 gRPC 服务，定义于 `autonomy/bridge/proto/external_command_service.proto`。

接入步骤见 [01 接入与验证](01_connection_guide.md)。

## 2.1 方法列表

| RPC | 分类 | 请求 → 响应 | 参考 |
|-----|------|-------------|------|
| `GetCapabilities` | Query | `Empty` → `Capabilities` | [04](04_query_api.md) |
| `GetRobotSnapshot` | Query | `Empty` → `RobotState` | [04](04_query_api.md) |
| `GetActiveTask` | Query | `Empty` → `ActiveTaskInfo` | [04](04_query_api.md) |
| `ReceiveBotStates` | Stream | `Empty` → stream `RobotState` | [05](05_stream_api.md) |
| `ReceiveBotEvents` | Stream | `Empty` → stream `RobotEvent` | [05](05_stream_api.md) |
| `EmergencyStop` | System | `EmergencyStopRequest` → `CommandAck` | [06](06_system_api.md) |
| `CancelAllTasks` | System | `CancelAllTasksRequest` → `CommandAck` | [06](06_system_api.md) |
| `SendNavigationCommand` | Command | `NavigationCommandRequest` → stream `*Response` | [07](07_navigation_command.md) |
| `SendFollowCommand` | Command | `FollowCommandRequest` → stream `*Response` | [09](09_follow_command.md) |
| `SendTeleopCommand` | Command | stream `TeleopCommandRequest` → stream `*Response` | [10](10_teleop_command.md) |
| `SendDockCommand` | Command | `DockCommandRequest` → stream `*Response` | [11](11_dock_command.md) |
| `SendMapCommand` | Command | `MapCommandRequest` → stream `*Response` | [12](12_map_command.md) |

## 2.2 RPC 分类

| 分类 | 用途 | 测试关注点 |
|------|------|------------|
| **Query** | 无副作用只读查询 | 连通性、能力矩阵、版本号 |
| **Stream** | 长连接状态/事件 | 频率、字段完整性、断线重连 |
| **System** | 全局急停/取消 | 抢占行为、幂等 |
| **Command** | 下发机载任务 | Stream 末帧、`TaskType` 互斥 |

## 2.3 流模式

| 模式 | 方法 | 客户端 |
|------|------|--------|
| Unary | Query / System | 单次 `Invoke` |
| Empty → Stream | `ReceiveBot*` | 循环 `Read()` |
| Unary → Stream | `Send*`（除 Teleop） | 发一帧 Request，读 Stream 至 `ack.final=true` |
| Bidi Stream | `SendTeleopCommand` | 双工读写 |

## 2.4 调用约束

- Command / System 请求首字段 **`header`**（[03 §3.1 RequestHeader](03_common_types.md#31-requestheader)）。
- Command 响应首字段 **`ack`**；**`ack.final=true`** 表示任务结束。
- **同一时刻仅一个 Command 任务**；`EmergencyStop` 可打断任意任务。
- MQTT Topic 与字段同名，见 [mqtt/04 §4.1](../mqtt/04_topic_protocol.md#41-topic-命名)。
- **Command 页写法**：先贴 `external_command_service.proto` 原文，再逐字段说明（[07](07_navigation_command.md) 起）。

## 2.5 Command 与 TaskType 对照

| TaskType | 值 | RPC | `supports_*` 门控 |
|----------|-----|-----|-------------------|
| `TASK_TYPE_NAVIGATION` | 1 | `SendNavigationCommand` | `supports_navigation` |
| `TASK_TYPE_FOLLOW` | 2 | `SendFollowCommand` | `supports_follow` |
| `TASK_TYPE_TELEOP` | 3 | `SendTeleopCommand` | `supports_teleop` |
| `TASK_TYPE_EXPLORATION` | 4 | `SendExplorationCommand` | `supports_exploration` |
| `TASK_TYPE_DOCK` | 5 | `SendDockCommand` | `supports_docking` |
| `TASK_TYPE_MAP` | 6 | `SendMapCommand` | `supports_map_management` |
