(async-grpc-handler-api)=
# Handler API

新增或改一个 RPC 的 C++ 入口。

### 三步注册

1. `DEFINE_HANDLER_SIGNATURE(Name, Req, Resp|Stream<Resp>, "/full.Method")`
2. `class H : public RpcHandler<Name> { void OnRequest(...) override; };`
3. `RegisterHandlers<H, ...>(builder);`

| 方法 | 用途 |
|------|------|
| `OnRequest` | 必须实现 |
| `Send` / `Finish` | Server stream 推送 / 结束 |
| `GetWriter()` | Action/worker 线程写 stream |
| `GetContext<T>()` | Bridge Context、Stub |

### 要点

| 主题 | 说明 |
|------|------|
| Unary | 通常一次 `Send` 或只 `Finish` |
| Stream | 先 ACK 再进度；busy 可首帧拒绝 |
| Context | `Build` 后、`Start` 前注入 ExecutionContext |
| 测试 | `RpcHandlerTestServer`、bridge unit tests |

宏定义见 `handler_templates.hpp`；勿在 Handler 里直接 pub Autolink topic（走 Stub）。
