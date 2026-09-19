# Bridge 文档索引

本目录是 **autonomy/bridge** 的权威长文：gRPC 业务四层之上的**平台横切能力**（policy / tools）、配置、安全、可观测性与测试。

短备忘（读代码前）：[`../grpc/DESIGN.md`](../grpc/DESIGN.md)。模块总览：[`../README.md`](../README.md)。

## 阅读顺序

| 顺序 | 文档 | 内容 |
|------|------|------|
| 1 | [01_grpc_platform_overview.md](01_grpc_platform_overview.md) | 业务四层 + 平台层；请求路径；不变量 |
| 2 | [02_policy.md](02_policy.md) | 鉴权、元数据、限流、deadline / 可靠性 |
| 3 | [03_tools.md](03_tools.md) | 拦截器链、Health、Reflection、Transport、bootstrap |
| 4 | [04_security_and_tls.md](04_security_and_tls.md) | mTLS、Bearer、威胁模型 |
| 5 | [05_observability_otel.md](05_observability_otel.md) | OpenTelemetry / OpenCensus 关系 |
| 6 | [06_configuration.md](06_configuration.md) | `GrpcOptions` / `bridge.pb.txt` 逐字段 |
| 7 | [07_async_grpc_extension.md](07_async_grpc_extension.md) | `async_grpc::Server::Builder` 扩展 |
| 8 | [08_testing.md](08_testing.md) | 单元 / 功能性测试矩阵与手工探针 |
| 9 | [09_cli_handbook.md](09_cli_handbook.md) | **CLI 全手册**：serve/call/list/describe + 全业务用例与**终端期望输出** |

## 源码树对照

```text
autonomy/bridge/
  policy/     # 决策：auth · metadata · rate_limit · reliability
  tools/      # 机制：interceptors · health · reflection · transport · otel · bootstrap
  grpc/       # 业务：Context · DomainBundle · Handlers · Stubs
  test/       # 单元 + 功能性测试（见 08_testing.md）
  docs/       # 本目录
```

## 设计原则

1. **Handler 保持薄** — 平台横切不进入 `rpc_*_handlers` 业务分支。
2. **policy 与 tools 分离** — policy 回答「允不允许」；tools 回答「怎么挂到 gRPC」。
3. **默认可降级** — Reflection / OTel / TLS 可关；缺库时 FEATURE 关闭不阻断默认构建。
4. **配置单一来源** — `proto/grpc_options.proto` + `conf/bridge.pb.txt`；语义以本目录为准。
