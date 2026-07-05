(async-grpc-upstream)=
# 上游参考

> [§4 gRPC 总览](../04_grpc.md) · 本专题 **grpc/06**（H2 **6.x**）。

对照 [cartographer-project/async_grpc](https://github.com/cartographer-project/async_grpc)（Apache-2.0）。Autonomy vendoring 至 `autonomy/common/async_grpc/`，命名空间 `autonomy::common::async_grpc`。

---

## 6.1 上游仓库

| 项目 | 说明 |
|------|------|
| 仓库 | [github.com/cartographer-project/async_grpc](https://github.com/cartographer-project/async_grpc) |
| 场景 | [Cartographer Cloud](https://github.com/cartographer-project/cartographer) 传感器流式上传 |
| Autonomy 扩展 | `BUILD_TRACING`、Bridge `GrpcBridgeServer` |

| Cartographer Cloud | Autonomy Bridge |
|--------------------|-----------------|
| Client-Stream 传感器上行 | `ReceiveBotStates` / `ReceiveBotEvents`（规划） |
| Server-Stream 结果 | `SendNavigationCommand` 进度 |
| `ExecutionContext` 共享 MapBuilder | `GrpcBridgeContextInterface` + Navigator（待接） |

## 6.2 设计动机

同步 gRPC server 每 RPC 一线程；机群 + 多 Stream 不可扩展。官方：固定 worker + libgrpc 异步 CQ + 自建 EQ + Handler 模型。

(async-grpc-getsquare)=
## 6.3 官方最小示例：GetSquare

上游 Unary 示例（[README](https://github.com/cartographer-project/async_grpc#example-server-offering-unary-rpc-method-getsquare)）：

```cpp
DEFINE_HANDLER_SIGNATURE(
    GetSquareSignature, proto::GetSquareRequest, proto::GetSquareResponse,
    "/proto.Math/GetSquare")

void OnRequest(const GetSquareRequest& req) override {
    auto r = std::make_unique<GetSquareResponse>();
    r->set_output(req.input() * req.input());
    Send(std::move(r));  // Unary：Send 即 Finish(OK)
}
```

Bridge 用法相同，见 [grpc/04 §4.1](04_handler_api.md#41-三步创建)。

(async-grpc-thread-guarantee)=
## 6.4 官方线程保证

单次 RPC 交换内 Handler **创建后始终在同一线程**执行 → **不必** thread-safe。跨 Rpc 共享 mutable 状态须 `ExecutionContext`。

## 6.5 Rpc 预创建

启动 idle Rpc 数：$N_{\text{idle}} = N_{\text{cq}} \times M$。Bridge：$4 \times 2 = 8$。

## 6.6 RpcEvent 与 WRITE_NEEDED

libgrpc 完成 op → CQ → PushToEventQueue → EQ。`Send()` / `GetWriter()` → `WRITE_NEEDED` **直注 EQ、绕过 CQ**。

## 6.7 Client

`Client<Signature>` 同步调用远端 RPC；`NavigatorStub` uplink 可复用。见 [client.h](https://github.com/cartographer-project/async_grpc/blob/master/async_grpc/client.h)。

## 6.8 RetryStrategy

`RetryStrategy` + `Client::Retry` 封装重试；uplink 弱网场景可选。

## 6.9 RpcHandlerTestServer

`testing/rpc_handler_test_server.h`：同步驱动 Handler 全生命周期，mock `ExecutionContext` 后单元测试 Bridge Handler（优于纯 grpcurl）。

## 6.10 Autonomy 扩展

| 能力 | 上游 | Autonomy |
|------|------|----------|
| Tracing | 可选 | `BUILD_TRACING=1` |
| MessageSize | — | Builder 可设 send/receive 上限 |
| Bridge | — | `GrpcBridgeServer` + `AutonomyService` |

**进一步阅读**：[上游 README](https://github.com/cartographer-project/async_grpc/blob/master/README.md) · [gRPC C++ Async](https://grpc.io/docs/languages/cpp/async/) · [grpc/05 Bridge 集成](05_bridge_integration.md)

---

**导航**：[← 05 Bridge 集成](05_bridge_integration.md) · [07 Handler 注册 →](07_handlers.md)
