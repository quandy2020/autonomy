# 03 · Tools（机制层）

Tools 把 policy 与 gRPC 运行时接在一起。实现位于 `autonomy/bridge/tools/`。

## 1. Bootstrap

```cpp
#include "autonomy/bridge/tools/bootstrap.hpp"

tools::ApplyPlatform(server_builder, grpc_options);
```

作用：按 `GrpcOptions` 配置 ChannelArgs、Credentials、Health、Reflection、拦截器工厂。由 `grpc/server.cpp` 在注册 Handler 后调用。

## 2. 拦截器链（固定顺序）

1. **Logging** — 方法名、`x-request-id`、goal 相关日志（尽力而为）
2. **Auth** — Bearer / TLS 模式
3. **Metadata** — 必填键
4. **RateLimit** — 令牌桶
5. **OTel** — 默认 no-op；`enable_opentelemetry=true` 且编译 FEATURE 时启用

任一环拒绝则后续不执行，RPC 以对应 Status 结束。

实现：`tools/interceptors/*` + `server_interceptor_chain.hpp`。

> 说明：gRPC C++ experimental interceptor 与 async_grpc 自定义完成队列模型的集成点在 `async_grpc::Server::Builder::AddInterceptorFactory`。若运行时版本不支持，bootstrap 记录警告并跳过链（见 [07](07_async_grpc_extension.md)）。

## 3. Health

- API：`tools/health/health_service.hpp` 包装 default health check。
- 选项：`enable_health_check`（默认 true）。
- 服务整体状态：`SERVING`（Start 成功后）；Shutdown 前可设 `NOT_SERVING`。
- 探针：

```bash
grpc_health_probe -addr=127.0.0.1:5005
```

## 4. Reflection

- 选项：`enable_server_reflection`（默认 **false**）。
- 依赖：`gRPC::grpc++_reflection`（CMake FEATURE `grpc_reflection`）。
- 用途：开发期 `grpcurl list` / `describe`。
- 风险：暴露全部服务与方法；**生产机载勿开**。

```bash
grpcurl -plaintext 127.0.0.1:5005 list
grpcurl -plaintext 127.0.0.1:5005 describe automsgs.rpcs.system.SystemService
```

## 5. Transport

`tools/transport/channel_args_builder.hpp`：

- `max_receive_message_bytes` / `max_send_message_bytes`
- keepalive：`keepalive_time_ms`、`keepalive_timeout_ms`、`permit_keepalive_without_calls`

`credentials_factory.hpp`：

- insecure（默认）
- TLS：读 `tls_cert_path` / `tls_key_path` / 可选 `tls_ca_path` + `tls_require_client_cert`

## 6. OTel

见 [05_observability_otel.md](05_observability_otel.md)。默认 `NoopTracerProvider`。

## 7. CLI 客户端（`rpc_probe`）

`tools/rpc_probe.*` 由 `autonomy.bridge list|describe|call` 调用。完整参数说明与**全部业务测试用例**见 [09_cli_handbook.md](09_cli_handbook.md)。
