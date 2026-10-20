(async-grpc-handler-api)=
# Handler API

### 4.1 三步

1. `DEFINE_HANDLER_SIGNATURE(Name, Req, Resp|Stream<Resp>, "/full.Method")`
2. `class H : public RpcHandler<Name> { void OnRequest(...) override; };`
3. `builder.RegisterHandler<H>();`

| 方法 | 用途 |
|------|------|
| `OnRequest` | 必须实现 |
| `Send` / `Finish` | 推送 / 结束 |
| `GetWriter()` | 跨线程写 |
| `GetContext<T>()` | ExecutionContext |

### 4.2 要点

| 主题 | 说明 |
|------|------|
| Unary | 单次 `Send` 即结束 |
| 跨线程 | 用 `GetWriter()`，勿裸 `Send` |
| Context | `Build` 后、`Start` 前 `SetExecutionContext` |
| 测试 | `RpcHandlerTestServer` |
