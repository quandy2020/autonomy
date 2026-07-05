(async-grpc-rpc-lifecycle)=
# Rpc 生命周期

> [§4 gRPC 总览](../04_grpc.md) · 本专题 **grpc/03**（H2 **3.x**）。

每个 gRPC 调用对应一个 `Rpc` 对象，由事件状态机驱动建连 → 读写 → 销毁。

---

## 3.1 Rpc 结构

| 成员 | 用途 |
|------|------|
| `server_context_` | deadline、元数据、取消 |
| `request_` / `response_` | Protobuf 缓冲 |
| `handler_` | `RpcHandler` 实例 |
| `send_queue_` | Write/Finish 串行队列 |
| `server_async_*` | 按 RPC 类型选用 Async 接口 |

## 3.2 四种 RPC 类型

| Incoming | Outgoing | 类型 | Bridge |
|----------|----------|------|--------|
| `Request` | `Response` | Unary | — |
| `Stream<Req>` | `Response` | Client Streaming | — |
| `Request` | `Stream<Resp>` | **Server Streaming** | `SendNavigationCommand` 等 |
| `Stream<Req>` | `Stream<Resp>` | Bidi | 可扩展 |

由 `type_traits.h` 中 `Stream<T>` 与 proto streaming 标志推断。

## 3.3 状态机（Server Streaming）

```
NEW_CONNECTION → OnConnection → READ(req) → OnRequest
    → WRITE_NEEDED → WRITE(resp)* → FINISH → DONE → Remove
```

Unary：`OnRequest` 后单次 `Send()` 等效响应 + 结束。

## 3.4 Write 队列

`Send()` / `Finish()` 入 `send_queue_`；EQ 线程 `HandleSendQueue()` 串行调用 libgrpc Async Write，保证顺序。

## 3.5 Handler 回调

| 回调 | Bridge 用途 |
|------|-------------|
| `OnRequest` | 解析 command、调 Navigator（**当前空实现**） |
| `OnReadsDone` | Client Stream 读完成（未用） |
| `OnFinish` | 清理资源 |

失败：`Finish(::grpc::Status(INVALID_ARGUMENT, "..."))`。

## 3.6 并发上界

在飞 Rpc：$N_{\text{active}} \lesssim N_{\text{methods}} \times (N_{\text{cq}} + \text{并发客户端})$。若 $T_{\text{queue}} = N_{\text{active}} \cdot T_h / N_{\text{event}}$ 超过 client deadline → `DEADLINE_EXCEEDED`。

---

**导航**：[← 02 双队列](02_dual_queue.md) · [04 Handler API →](04_handler_api.md)
