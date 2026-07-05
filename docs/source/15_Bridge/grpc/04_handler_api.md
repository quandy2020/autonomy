(async-grpc-handler-api)=
# Handler API

> [§4 gRPC 总览](../04_grpc.md) · 本专题 **grpc/04**（H2 **4.x**）。

基于 `async_grpc` 编写 RPC Handler；Bridge 参考 `SendNavigationHandler`。

---

## 4.1 三步创建

**Step 1 — Signature**

```cpp
DEFINE_HANDLER_SIGNATURE(
    SendNavigationSignature,
    proto::NavigationCommandRequest,
    autonomy::common::async_grpc::Stream<proto::NavigationCommandResponse>,
    "/autonomy.bridge.proto.AutonomyService/SendNavigationCommand")
```

| 写法 | 含义 |
|------|------|
| `MyRequest` | Unary 请求 |
| `Stream<MyResponse>` | Server Streaming 响应 |

**Step 2 — Handler**

```cpp
class SendNavigationHandler
    : public RpcHandler<SendNavigationSignature> {
public:
    void OnRequest(const proto::NavigationCommandRequest& request) override;
};
```

| 方法 | 用途 |
|------|------|
| `OnRequest` | **必须实现** |
| `Send(unique_ptr<Response>)` | Stream 推送 |
| `Finish(Status)` | 结束 RPC |
| `GetWriter()` | 跨线程安全 Write |
| `GetContext<T>()` | 加锁访问 ExecutionContext |

**Step 3 — 注册**

```cpp
builder.RegisterHandler<SendNavigationHandler>();
```

`RegisterHandler` 校验 proto Descriptor 与 Stream 类型一致。

## 4.2 Send / Finish / Writer

- `Send()` → `rpc_->Write()` → `WRITE_NEEDED` → EQ 串行写出
- Unary：单次 `Send(response)` 即响应并结束
- Navigator 回调线程应 `GetWriter().Write()`，勿裸 `Send()`（避免 UAF）

## 4.3 ExecutionContext

```cpp
server->SetExecutionContext(std::make_unique<GrpcBridgeContextInterface>());
// Build() 之后、Start() 之前

void OnRequest(const Request& req) {
    GetContext<GrpcBridgeContextInterface>()->navigator_stub()->SendGoal(...);
}
```

## 4.4 Server Streaming 模板（规划）

```cpp
void SendNavigationHandler::OnRequest(const Request& request) {
    switch (request.command()) {
        case NAV_CMD_START:
            if (request.goals().empty()) {
                Finish(Status(INVALID_ARGUMENT, "goals empty")); return;
            }
            Send(make_response(/* ack accepted */));
            GetWriter().Write(...);  // Navigator 进度 → ack.final=false
            break;
        case NAV_CMD_CANCEL:
            GetContext<Ctx>()->navigator_stub()->Cancel();
            Finish(Status::OK);
            break;
        default:
            Finish(Status(FAILED_PRECONDITION, "invalid state"));
    }
}
```

完整 walkthrough：[grpc/05](05_bridge_integration.md) · 上游 GetSquare：[grpc/06 §6.3](06_upstream_reference.md#63-官方最小示例getsquare)。

## 4.5 要点

| 主题 | 说明 |
|------|------|
| RegisterHandler | method 全名须与 proto 一致，否则 `CHECK` 失败 |
| 测试 | `RpcHandlerTestServer` mock Context（[grpc/06 §6.9](06_upstream_reference.md#69-rpchandlertestserver)） |
| 客户端 | uplink 用 `Client<Signature>` + `RetryStrategy`（[grpc/06 §6.7](06_upstream_reference.md#67-client)） |
| 陷阱 | Stream 须 `Finish`；`SetExecutionContext` 须在 `Start` 前；跨线程用 `GetWriter` |

---

**导航**：[← 03 Rpc 生命周期](03_rpc_lifecycle.md) · [05 Bridge 集成 →](05_bridge_integration.md)
