(grpc-handlers)=
# Handler 注册与规划

> [§4 gRPC 总览](../04_grpc.md) · **grpc/07** · H2 **7.x**。

`automsgs.rpcs.*` Handler 实现状态的单一维护点。契约见 [rpcs/02 服务概述](../rpcs/02_service_overview.md) · 接线见 [05 Bridge 集成](05_bridge_integration.md)。注册入口：`grpc/server.cpp` → `RegisterRpcHandlers`。

## 7.1 域 Handler 对照（摘要）

| 服务 | Handler 文件 | Stub | 代表 RPC |
|------|--------------|------|----------|
| `NavigationService` | `rpc_navigation_handlers` | `NavigatorStub` | `Navigate` · `Pause` · `Resume` · `Replan` · `Cancel` · `GetStatus` |
| `FollowService` | `rpc_follow_handlers` | `FollowStub` | `Follow` · `Pause` · `Resume` · `Cancel` · `GetStatus` |
| `TeleopService` | `rpc_teleop_handlers` | `TeleopStub` | `Velocity` · `DriveOnHeading` · `BackUp` · `Spin` · 生命周期 |
| `ChargeService` | `rpc_charge_handlers` | `ChargeStub` | `Return` · `Leave` · 生命周期 |
| `MapService` | `rpc_map_handlers` | `MapServiceStub` / `MappingStub` | `StartMapping` · `FinishMapping` · `CancelMapping` · `GetMappingStatus` · `ListMaps` · `GetMap` · `GetMapMetadata` · `SaveMap` · `DeleteMap` · `SetCurrentMap` |
| `ExplorationService` | `rpc_explore_handlers` | `ExplorationStub` | `Explore` · `SetArea` · `SaveMap` · 生命周期 |
| `NavigationService` | `rpc_navigation_handlers` | `NavigatorStub` → `NavigationTask` | `Navigate` · lifecycle |
| `ExplorationService` | `rpc_explore_handlers` | `ExplorationStub` → `ExplorationTask` | `Explore` · lifecycle · SetArea · SaveMap |
| `LocalizationService` | `rpc_localization_handlers` | `LocalizationStub` | `GetPose` · `GetStatus` · `SetInitialPose` |
| `SensorService` | `rpc_sensor_handlers` | `SensorStub` | `ListSensors` · `GetSample` · `GetParameters` · `SetParameters` · `SaveParameters` · `LoadParameters` · `Record` · `CancelRecord` · `GetRecordStatus` |
| `SystemService` | `rpc_system_handlers` | `SystemMonitorStub` / Hub / profile | `Heartbeat` · `GetInfo` · `GetStatus` · `GetHealth` · `GetRobotFullInfo` · `EmergencyStop` · `ClearEmergencyStop` · `CancelAllGoals` · `GetActiveGoal` · `GetCapabilities` |

## 7.1.1 模板约定

| 层 | 设施 | 用法 |
|----|------|------|
| Handler | `rpc_<domain>_handlers.*` | **同域多 class 同文件**；优先 `BRIDGE_STREAM` / `LIFECYCLE` / `UNARY` / `DECL` |
| Handler 工具 | `handlers/util.hpp` · `handler_templates.hpp` | `RequireContext` / Unary reply / `RelayStream` / `BRIDGE_LIFECYCLE` |
| Goal 通道 | `goal_channel_stub.hpp` · `goal_channel_command_stub.hpp` · `BRIDGE_CHANNEL_TRAITS` | Follow / Charge / Mapping / TeleopVel |
| Action | `teleop_stub.hpp`（`teleop::*Traits`） | Teleop 相对运动（Drive / BackUp / Spin；Nav 已迁 GoalChannel） |
| 命令分发 | `command_dispatch.hpp` | `DispatchCommands` + `function_traits` |
| 最新消息 | `latest_message_cache.hpp` | Localization / MapService |
| 传感器 | `sample_cache.hpp` · `proto_pool.hpp` | Sample + Record 帧 `ProtoPool` |

设计不变量与失败路径：源码旁 `grpc/DESIGN.md`。

## 7.2 Handler 签名示例

```cpp
DEFINE_HANDLER_SIGNATURE(
    NavigateSignature,
    ::automsgs::rpcs::navigation::NavigateRequest,
    async_grpc::Stream<::automsgs::rpcs::navigation::NavigateResponse>,
    "/automsgs.rpcs.navigation.NavigationService/Navigate")
```

## 7.3 互斥与 Stream 约定

`TaskMuxer` 保证互斥 Command；`EmergencyStop` / `CancelAllGoals` 可抢占。时序见 [rpcs/03](../rpcs/03_common_types.md)。

命令流 Handler：

1. 校验 `goal_id` / 请求字段
2. 立即 ACK（非终态）后卸荷到 Stub / Session
3. 进度帧同 `goal_id`
4. 终态帧后 `Finish`

Teleop `Velocity` Bidi：`OnRequest` 处理速度流；看门狗超时停速。

## 7.4 gRPC 状态码

与 [§4.4](../04_grpc.md#44-状态码与调优) 及 `automsgs.rpcs.common.Status` 一致。
