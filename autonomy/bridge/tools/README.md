# Tools — Bridge gRPC 平台机制层

将 `policy/` 决策接到 `async_grpc::Server::Builder`。

| 目录 | 职责 |
|------|------|
| `bootstrap.hpp` | `ApplyPlatform` 入口 |
| `transport/` | ChannelArgs / Credentials |
| `health/` | Health 状态辅助 |
| `reflection/` | Reflection 库探测 |
| `interceptors/` | Logging→Auth→Metadata→RateLimit→OTel |
| `otel/` | TracerProvider（默认 no-op） |

详细说明见 [`../docs/03_tools.md`](../docs/03_tools.md)。
