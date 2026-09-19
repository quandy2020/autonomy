# 01 · gRPC 平台总览

## 1. 两层架构

Bridge 在机载进程内同时承担：

| 层 | 职责 | 目录 |
|----|------|------|
| **业务四层** | Handler → Context/DomainBundle → Stub → Channel/Action | `grpc/` |
| **平台横切** | 拦截器、健康检查、反射、元数据、鉴权、限流、传输参数、可观测性 | `policy/` + `tools/` |

业务层不变量见 [`../grpc/DESIGN.md`](../grpc/DESIGN.md)。平台层不改变 `automsgs.rpcs.*` 表面，只改变**进入 Handler 之前**的准入与旁路服务（Health / Reflection）。

## 2. 请求路径

```text
gRPC Client
    │  metadata / auth / deadline
    ▼
ServerInterceptor chain   (tools/interceptors)
    │  Logging → Auth → Metadata → RateLimit → OTel
    ▼
async_grpc RpcHandler     (grpc/handlers)
    │
    ▼
Context / DomainBundle / Stub
```

旁路（不经过业务 Handler）：

- **Health**：`grpc.health.v1.Health`
- **Reflection**：`grpc.reflection.v1alpha.ServerReflection`（默认关闭）

## 3. 启动装配

[`tools/bootstrap.hpp`](../tools/bootstrap.hpp) 的 `ApplyPlatform(Builder&, GrpcOptions)` 在 [`grpc/server.cpp`](../grpc/server.cpp) 中于 `RegisterRpcHandlers` 之后、`Build()` 之前调用，顺序：

1. ChannelArgs（消息大小、keepalive）
2. ServerCredentials（insecure 或 TLS）
3. Health / Reflection 开关
4. Interceptor factories（按固定链序）

## 4. 不变量

1. 缺 Context 仍由 Handler util 返回 `UNKNOWN` / `INTERNAL`（业务路径不变）。
2. Estop / CancelAll 仍走 DomainBundle；拦截器**不**替代业务取消。
3. Reflection 默认 **off**（避免机载暴露服务清单）；Health 默认 **on**（运维探针）。
4. Bearer / Metadata 拒绝使用 gRPC 状态码（见 [02_policy.md](02_policy.md) 拒绝码表），不写业务 Status protobuf。

## 5. 相关文档

- Policy：[02_policy.md](02_policy.md)
- Tools：[03_tools.md](03_tools.md)
- 配置：[06_configuration.md](06_configuration.md)
