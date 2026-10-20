(grpc-handlers)=
# Handler 注册

**入口：** `grpc/server.cpp` 里 `RegisterRpcHandlers` → `register_handlers.hpp`。
契约表：[rpcs/02](../rpcs/02_service_overview.md)。

**什么时候看：** 确认某 RPC 是否已实现、或新增 Handler 放哪注册。

### 域对照

| 服务 | Handler 文件 | 代表 RPC |
|------|--------------|----------|
| `NavigationService` | `rpc_navigation_handlers` | Navigate / Pause / Resume / Replan / Cancel / GetStatus |
| `FollowService` | `rpc_follow_handlers` | Follow · 生命周期 |
| `TeleopService` | `rpc_teleop_handlers` | Velocity · DriveOnHeading · BackUp · Spin |
| `ChargeService` | `rpc_charge_handlers` | Return / Leave |
| `MapService` | `rpc_map_handlers` | Start/Finish Mapping · List/Get/Save Map |
| `ExplorationService` | `rpc_explore_handlers` | Explore · SetArea · SaveMap |
| `VoiceService` | `rpc_voice_handlers` | Execute · Cancel |
| `LocalizationService` | `rpc_localization_handlers` | GetPose · SetInitialPose |
| `SensorService` | `rpc_sensor_handlers` | ListSensors · GetSample · Record |
| `SystemService` | `rpc_system_handlers` | Heartbeat · Get* · Estop · CancelAll · Caps |

### 模板与设施

| 层 | 设施 |
|----|------|
| Handler | `BRIDGE_STREAM` / `LIFECYCLE` / `UNARY` 宏 |
| Goal 通道 | `goal_channel_*`、`NavigatorStub` |
| 互斥 | `TaskMuxer` + session gate（estop、busy、duplicate cmd_id） |
| 缓存 | `latest_message_cache`、`sample_cache`、`ProtoPool` |

典型流：校验 → 立即 ACK 或拒绝帧 → Stub 卸荷 → 进度帧 → 终态 `Finish`。  
Estop / CancelAll 经 `CancelRegistry` 抢占。

### 接线注意

Stub 未接 Autolink/Action 时会出现 **RPC 有响应但下游无动作**；联调以 task 进程日志为准。  
Capabilities `supports_*=false` 时客户端仍可能连上服务，但业务应拒绝或降级。

### 常见坑

| 现象 | 说明 |
|------|------|
| list 有服务 Navigate 无进度 | 只注册了 Handler，Stub 未通 |
| 重复 RegisterHandlers | 模板包重复类型可能双注册 |
| BUILD_GRPC=OFF | 整节代码未编入二进制 |
