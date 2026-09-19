(bridge-rpc-protocol)=
# 3. RPC 协议

Bridge 对外 **仅** 注册 `automsgs.rpcs.*`（见 `automsgs/proto/rpcs/`）。`autonomy/bridge/proto` 只保留机载配置：

| 文件 | 内容 |
|------|------|
| `bridge/proto/*_options.proto` | 机载配置（非 RPC；一消息一文件）→ [§1](01_options.md) |

| 包 | 域服务 |
|----|--------|
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

**集成 / 调 API**：从侧边栏 [01 接入与验证](rpcs/01_connection_guide.md) 跑通 `rpc-cli.py` / `grpcurl`；全量用例见 [13 集成测试](rpcs/13_integration_tests.md)。

**服务端实现**：[§4 gRPC](04_grpc.md) · [grpc/07 Handlers](grpc/07_handlers.md)

---

**导航**：[← §2 架构](02_architecture.md) · [rpcs/ API →](rpcs/index.rst)
