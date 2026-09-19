# 07 · async_grpc Server::Builder 扩展

## 1. 动机

历史 Cartographer `async_grpc::Server` 仅支持地址、线程、消息大小与 OpenCensus tracing，并在构造时写死 `InsecureServerCredentials`。Bridge 平台需要：

- 可配置 Credentials（TLS）
- ChannelArgs（keepalive 等）
- Default Health Check
- Proto Reflection（可选）
- Server interceptor factories

因此在 **不破坏现有 RegisterHandler API** 的前提下扩展 Builder / Options。

## 2. 新增 API（摘要）

| 方法 | 作用 |
|------|------|
| `AddChannelArguments(args)` | 设置 `grpc::ChannelArguments` |
| `SetServerCredentials(creds)` | 覆盖默认 insecure |
| `EnableDefaultHealthCheckService(bool)` | 启用 health |
| `EnableProtoReflection(bool)` | 启用 reflection（需库） |
| `AddInterceptorFactory(factory)` | 追加 interceptor factory |

调用方：`tools::ApplyPlatform`。

## 3. 生命周期

1. `Builder` 收集 options + handlers + platform 设置  
2. `Build()` → `new Server(options, interceptors)` 注册 service  
3. `Server` 构造：`AddListeningPort`、ChannelArgs、Health/Reflection 插件、interceptors  
4. `Start()` → `BuildAndStart()`

## 4. 兼容性

- 未调用新 API 时行为与扩展前一致（insecure + 默认消息大小）。
- Reflection 宏 `AUTONOMY_HAVE_GRPC_REFLECTION` 未定义时，`EnableProtoReflection(true)` 仅打日志。
- CMake FEATURES：`grpc_reflection`（链接 `gRPC::grpc++_reflection` 并定义上述宏）、`otel`（可选 SDK；缺省 no-op）。

## 5. 线程安全

Builder 仅在单线程构造期使用；与现有一致。
