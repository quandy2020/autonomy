(bridge-rpc-protocol)=
# 3. RPC 协议

对外客户端与 Bridge 之间的**唯一契约**是 `automsgs.rpcs.*`（`automsgs/proto/rpcs/`）。`autonomy/bridge/proto` 仅描述机载 Bridge **配置**（见 [§1](01_options.md)）。

按 **服务域** 拆 proto 包：导航/跟随/遥操等 Command 多为 **server-stream**；系统查询多为 **Unary**。写客户端前先 `GetCapabilities`，再查对应 Command 页，避免对着未实现域发 RPC。

### 服务包一览

| 包 | 服务 |
|----|------|
| `automsgs.rpcs.navigation` | `NavigationService` |
| `automsgs.rpcs.follow` | `FollowService` |
| `automsgs.rpcs.teleop` | `TeleopService` |
| `automsgs.rpcs.charge` | `ChargeService` |
| `automsgs.rpcs.mapping` | `MapService` |
| `automsgs.rpcs.exploration` | `ExplorationService` |
| `automsgs.rpcs.voice` | `VoiceService` |
| `automsgs.rpcs.localization` | `LocalizationService` |
| `automsgs.rpcs.sensor` | `SensorService` |
| `automsgs.rpcs.system` | `SystemService` |

### 入口链接

| 目的 | 文档 |
|------|------|
| grpcurl / 连接 | [rpcs/01](rpcs/01_connection_guide.md) |
| Handler 实现 | [§4](04_grpc.md)、[grpc/07](grpc/07_handlers.md) |
| 联调顺序 | [rpcs/12](rpcs/12_integration_tests.md) |

### 常见坑

| 现象 | 说明 |
|------|------|
| `BUILD_GRPC=OFF` | 无 bridge 二进制与 5005 监听 |
| Capabilities 与 Task 不一致 | Command 被拒或 BUSY |
| 找统一 BotEvents stream | 已移除；用各 Command stream |
