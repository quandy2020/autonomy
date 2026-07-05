(async-grpc-dual-queue)=
# 双队列

> [§4 gRPC 总览](../04_grpc.md) · 本专题 **grpc/02**（H2 **2.x**）。

`async_grpc` 核心：**Completion Queue（CQ）** 与 **Event Queue（EQ）** 分离 — gRPC I/O 与 Handler 业务分属不同线程池，经 `RpcEvent` 传递。

---

## 2.1 动机与职责

同步 gRPC server 每 RPC 一线程；Bridge 多客户端并发 `SendNavigationCommand`（Server Streaming）不可接受。官方在 libgrpc 异步 CQ 之上增加 EQ，用固定 $N_{\text{cq}} + N_{\text{eq}}$ 线程 Multiplex 连接。

| 队列 | 线程 | 职责 | 阻塞容忍 |
|------|------|------|----------|
| Completion Queue | `num_grpc_threads` | 网络 I/O、Protobuf 编解码 | **极低** |
| Event Queue | `num_event_threads` | `RpcHandler` 业务 | 中等（可等 Navigator） |

**单 Rpc 单线程**：同一 RPC 内 Handler 不必 thread-safe；跨 Rpc 共享状态用 `ExecutionContext`（[grpc/06 §6.4](06_upstream_reference.md#64-官方线程保证)）。

## 2.2 数据流

```
Client ──HTTP/2──► CQ 线程 (Next → PushToEventQueue)
                      │
                      ▼
                 EQ 线程 (Pop → RpcEvent::Handle → OnRequest / Send / Finish)
                      │
                      ▼
              SendNavigationHandler → Navigator
```

`Send()` / `GetWriter()` 触发 `WRITE_NEEDED`，**直注 EQ、绕过 CQ**（[grpc/06 §6.6](06_upstream_reference.md#66-rpcevent-与-write_needed)）。

## 2.3 Server 启动

`Server::Start()`（`server.cpp`）：`BuildAndStart()` → 各 `Service::StartServing` → 预注册 idle Rpc（$N_{\text{idle}} = N_{\text{cq}} \times M_{\text{methods}}$）→ 启动 EQ 线程 → 启动 CQ 线程。

Bridge 默认 2 方法 × 4 CQ = **8** 个常驻 idle Rpc。

## 2.4 线程模型

| 线程池 | 循环 | 关键调用 |
|--------|------|----------|
| CQ | `completion_queue->Next()` | `PushToEventQueue()` |
| EQ | `event_queue->PopWithTimeout(100ms)` | `rpc_event->Handle()` |

`EventQueueSelector` Round-Robin 将新 Rpc 绑定到某一 EQ，保证单 Rpc 生命周期内线程固定。

## 2.5 事件类型

| Event | 来源 | 含义 |
|-------|------|------|
| `NEW_CONNECTION` | CQ | 客户端发起 RPC |
| `READ` | CQ | 收到请求（Client/Bidi Stream） |
| `WRITE_NEEDED` | EQ 内部 | `Send()` / `Finish()` 触发 |
| `WRITE` / `FINISH` / `DONE` | CQ | Write/Finish 完成、连接销毁 |

## 2.6 ActiveRpcs

`Service` 维护 `active_rpcs_`：`HandleNewConnection` 时 `OnConnection()` 实例化 Handler → `Clone()` 预注册下一 idle Rpc → `RequestNextMethodInvocation()`。

## 2.7 吞吐与线程配置

Event 线程吞吐 $\mu \approx N_{\text{event}} / T_h$。Bridge 建议 $N_{\text{event}} \geq N_{\text{grpc}}$，$N_{\text{grpc}} \approx \lceil \text{CPU}/2 \rceil$（见 [§4.4](../04_grpc.md#44-状态码与调优)）。

## 2.8 Shutdown

`StopServing` → `server_->Shutdown()` → join CQ → join EQ。`GrpcBridgeServer` 另 join `task_thread_`。

---

**导航**：[← 01 async_grpc](01_overview.md) · [03 Rpc 生命周期 →](03_rpc_lifecycle.md)
