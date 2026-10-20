(bridge-rpc-protocol)=
# 3. RPC 协议

对外 **仅** `automsgs.rpcs.*`（`automsgs/proto/rpcs/`）。`autonomy/bridge/proto` 只保留机载配置 → [§1](01_options.md)。

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

调 API：[rpcs/01](rpcs/01_connection_guide.md) · 用例索引：[rpcs/13](rpcs/12_integration_tests.md) 
服务端：[§4](04_grpc.md) · [grpc/07](grpc/07_handlers.md)
