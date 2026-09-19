# 05 · 可观测性与 OpenTelemetry

## 1. 与现有 OpenCensus 的关系

`autonomy/common/async_grpc` 内置 **OpenCensus** Stackdriver 导出（`BUILD_TRACING` + `EnableTracing()`）。Bridge 平台层新增的 **OpenTelemetry** 路径与之**并存、默认关闭**，避免强制新依赖。

| 路径 | 开关 | 默认 |
|------|------|------|
| async_grpc OpenCensus | `Builder::EnableTracing()` + 编译 `BUILD_TRACING` | off |
| Bridge OTel interceptor | `GrpcOptions.enable_opentelemetry` + FEATURE `opentelemetry` | off |

## 2. Span 约定（启用 OTel 时）

- Span 名：gRPC full method（如 `/automsgs.rpcs.system.SystemService/Heartbeat`）
- 属性：`rpc.system=grpc`、`rpc.service`、`rpc.method`、可选 `x-request-id`
- 状态：拦截器拒绝 → Error；业务 Finish OK → Ok（应用 Status 在属性中可选记录）

## 3. 导出

- `otel_service_name`：资源 `service.name`
- `otel_exporter_endpoint`：OTLP/gRPC 端点（如 `http://127.0.0.1:4317`）

第一期实现以 **接口 + Noop** 为主；真实 SDK 在 FEATURE 打开且库存在时链接。

## 4. Logging 拦截器

不依赖 OTel：始终可用，写 AINFO/AWARN（方法、request-id、拒绝原因）。
