(grpc-handlers)=
# Handler 注册

注册入口：`grpc/server.cpp` → `RegisterRpcHandlers`。契约：[rpcs/02](../rpcs/02_service_overview.md)。

### 7.1 域对照

| 服务 | Handler 文件 | 代表 RPC |
|------|--------------|----------|
| `NavigationService` | `rpc_navigation_handlers` | Navigate / Pause / Resume / Replan / Cancel / GetStatus |
| `FollowService` | `rpc_follow_handlers` | Follow · 生命周期 |
| `TeleopService` | `rpc_teleop_handlers` | Velocity · DriveOnHeading · BackUp · Spin |
| `ChargeService` | `rpc_charge_handlers` | Return / Leave |
| `MapService` | `rpc_map_handlers` | Start/Finish Mapping · List/Get/Save/Delete Map |
| `ExplorationService` | `rpc_explore_handlers` | Explore · SetArea · SaveMap |
| `LocalizationService` | `rpc_localization_handlers` | GetPose · SetInitialPose |
| `SensorService` | `rpc_sensor_handlers` | ListSensors · GetSample · Record |
| `SystemService` | `rpc_system_handlers` | Heartbeat · Get* · Estop · CancelAll · Caps |

### 7.2 模板

| 层 | 设施 |
|----|------|
| Handler | `BRIDGE_STREAM` / `LIFECYCLE` / `UNARY` |
| Goal 通道 | `goal_channel_*` · `BRIDGE_CHANNEL_TRAITS` |
| 分发 | `DispatchCommands` |
| 缓存 | `latest_message_cache` · `sample_cache` · `ProtoPool` |

命令流：校验 → 立即 ACK → Stub 卸荷 → 进度帧 → 终态 `Finish`。 
互斥：`TaskMuxer`；Estop / CancelAll 可抢占。
