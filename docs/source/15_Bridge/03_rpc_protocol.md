(bridge-rpc-protocol)=
# 3. RPC 协议

`autonomy/bridge/proto` 定义 Bridge 对外 gRPC API，包名 **`autonomy.bridge.proto`**。

| 文件 | 内容 |
|------|------|
| `external_command_service.proto` | `AutonomyService`（13 RPC） |
| `bridge_options.proto` | 机载配置（非 RPC）→ [§1](01_options.md) |

**集成 / 调 API**：从侧边栏 [01 接入与验证](rpcs/01_connection_guide.md) 跑通 `grpcurl`；全量用例见 [13 集成测试](rpcs/13_integration_tests.md)。

**服务端实现**：[§4 gRPC](04_grpc.md) · [grpc/07 Handlers](grpc/07_handlers.md)

---

**导航**：[← §2 架构](02_architecture.md) · [rpcs/ API →](rpcs/index.rst)
