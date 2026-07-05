(async-grpc-overview)=
# async_grpc

> [§4 gRPC 总览](../04_grpc.md) · 本专题 **grpc/01**（H2 **1.x**）。文档地图见 [00_guide §0.1](../00_guide.md#01-文档地图)。

## 1.1 定位

| 维度 | 说明 |
|------|------|
| Autonomy 路径 | `autonomy/common/async_grpc/` |
| 上游 | [cartographer-project/async_grpc](https://github.com/cartographer-project/async_grpc)（Apache-2.0） |
| 场景 | Cartographer Cloud 流式上传；Bridge 并发 Server Streaming |
| 消费者 | `autonomy/bridge/plugins/grpc`（`AutonomyService`） |

原生 gRPC 同步 server **每 RPC 一线程**；`async_grpc` 在 `CompletionQueue` 上封装 **Rpc 事件 + Handler 回调**，用固定 CQ/EQ 线程池 Multiplex 连接。相对手写异步 tag 状态机，提供 `RpcHandler` 抽象。

## 1.2 设计目标

1. **双队列**：CQ 跑 I/O 与编解码；EQ 跑 Handler（见 [grpc/02](02_dual_queue.md)）
2. **单 Rpc 单线程**：同一次 RPC 内 Handler 不必 thread-safe（[grpc/06 §6.4](06_upstream_reference.md#64-官方线程保证)）
3. **四种 RPC**：`Stream<T>` 表达流式方向（[grpc/03](03_rpc_lifecycle.md)）
4. **跨线程写**：`GetWriter()` + `WRITE_NEEDED` 直注 EQ
5. **共享上下文**：`ExecutionContext` / `GetContext<T>()`
6. **可选追踪**：`BUILD_TRACING=1` + OpenCensus

## 1.3 架构类图

| 类 / 组件 | 职责 |
|-----------|------|
| `Server` | CQ/EQ 线程池、Handler 注册表、`ExecutionContext` |
| `Service` | 路由事件到 `Rpc`；`ActiveRpcs` |
| `Rpc` | 单次 method 状态机；`Send` / `Finish` 队列 |
| `RpcHandler` | 用户业务；`OnRequest` / `Send` / `GetWriter` |
| `RpcEvent` | CQ ↔ EQ 事件载体 |

```{figure} images/diagram.png
:alt: async_grpc 框架架构类图
:align: center
:width: 88%
:name: fig-async-grpc-architecture

async_grpc 架构概览（图源 [上游 README](https://github.com/cartographer-project/async_grpc#overview)）。**RpcEvent** 经 **Completion Queue**（libgrpc）进入 **Event Queue**（Handler）。详见 [grpc/02 双队列](02_dual_queue.md)。
```

Bridge：`AutonomyService` → `SendNavigationHandler` 等位于 **Rpc → RpcHandler** 链路（[grpc/05 Bridge 集成](05_bridge_integration.md)）。

## 1.4 源码结构

```
autonomy/common/async_grpc/
├── server.h/cpp, service.h/cpp, rpc.h/cpp
├── rpc_handler.h, rpc_handler_interface.h
├── type_traits.h              # DEFINE_HANDLER_SIGNATURE
├── execution_context.h
├── completion_queue_thread.*, event_queue_thread.*
├── client.h, async_client.h, retry.h/cpp
└── testing/rpc_handler_test_server.h
```

## 1.5 Bridge 实例化

```
Server::Builder → RegisterHandler<SendNavigationHandler>()
    → Server (CQ×N + EQ×M + AutonomyService + ExecutionContext)
        → Rpc → SendNavigationHandler
```

构建与接线：[§4.2](../04_grpc.md#42-服务器构建) · [grpc/05 §5.7](05_bridge_integration.md#57-配置-server-参数映射)。

## 1.6 默认常量

| 常量 | 值 | 位置 |
|------|-----|------|
| `kDefaultMaxMessageSize` | 10 MB | `server.h` |
| `kPopEventTimeout` | 100 ms | `server.cpp` |
| Bridge `kMaxMessageSize` | 100 MB | `grpc_bridge.cpp` |
| 默认凭证 | `InsecureServerCredentials` | `server.cpp` |

---

**导航**：[← §4 总览](../04_grpc.md) · [02 双队列 →](02_dual_queue.md)
