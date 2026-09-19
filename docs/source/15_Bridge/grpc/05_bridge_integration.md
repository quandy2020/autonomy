(async-grpc-bridge-integration)=
# Bridge 集成

> [§4 gRPC 总览](../04_grpc.md) · 本专题 **grpc/05**（H2 **5.x**）。

`autonomy/bridge` 源码链路：**配置 → BridgeServer → Server → async_grpc → Handler → Proto**。

---

## 5.1 集成总览

```
bridge.lua / BridgeOptions
    → BridgeServer → Server
        → Server::Builder → Server (automsgs.rpcs)
            → SendNavigationHandler / SendExplorationHandler
```

框架背景：[cartographer async_grpc](https://github.com/cartographer-project/async_grpc) · 类图：[grpc/01](01_overview.md)。

## 5.2 启动链路

**BridgeServer**（`bridge_server.cpp`）：

```cpp
BridgeServer::BridgeServer() {
    grpc_bridge_ = std::make_unique<Server>();
}
void BridgeServer::Start() {
    grpc_bridge_->Start();
}
```

**Server** 构造（`server.cpp`）：

```cpp
Server::Builder b;
b.SetServerAddress("127.0.0.1");  // 待读 GrpcOptions，缺 port
b.SetNumGrpcThreads(4);
b.SetNumEventThreads(4);
b.SetMaxSendMessageSize(100 * 1024 * 1024);
b.RegisterHandler<SendNavigationHandler>();
b.RegisterHandler<SendExplorationHandler>();
grpc_server_ = b.Build();
```

**Start**：`task_thread_`（传感器占位）+ `grpc_server_->Start()`（CQ/EQ 线程池）。

| 线程 | 数量 | 职责 |
|------|------|------|
| CQ | 4 | gRPC I/O |
| EQ | 4 | Handler |
| `task_thread_` | 1 | 上行队列占位 |

## 5.3 已注册 RPC

Proto 定义 **13** 个 RPC；源码当前仅注册 2 个 Handler：

| Handler | gRPC Method | 类型 | 状态 |
|---------|-------------|------|------|
| `SendNavigationHandler` | `SendNavigationCommand` | Unary → Stream | ⏳ 已注册，`OnRequest` 空 |
| `SendExplorationHandler` | `SendExplorationCommand` | Unary → Stream | ⏳ 已注册，`OnRequest` 空 |
| — | 其余 13 RPC | 见 [grpc/07_handlers](../grpc/07_handlers.md) | ❌ 未 RegisterHandler |

Proto 定义：[rpcs/02 服务概述](../rpcs/02_service_overview.md) · Handler 签名：[grpc/04 §4.1](04_handler_api.md#41-三步创建)。

## 5.4 调用时序（SendNavigationCommand）

```
Client → CQ(NEW_CONNECTION) → EQ(OnRequest) → [Navigator SendGoal]
    → EQ(Send progress…) → CQ(Write) → Client
    → EQ(Finish) → DONE
```

当前 `OnRequest` 为空；规划见 [grpc/04 §4.4](04_handler_api.md#44-server-streaming-模板规划)。

## 5.5 Context（ExecutionContext）

`Context` 继承 `async_grpc::ExecutionContext`，持有 Muxer / StateHub / DomainBundle，
并共享 `WorkScheduler`。须在 `Build()` 后、`Start()` 前 `SetExecutionContext`。

细节与不变量：源码旁 `grpc/DESIGN.md`。

## 5.6 待实现 RPC

按 [grpc/07_handlers](../grpc/07_handlers.md#71-handler-对照表) 优先级：

| 优先级 | RPC | 说明 |
|--------|-----|------|
| P0 | `ReceiveBotStates` / `ReceiveBotEvents` | Empty → Stream；`vehicle_msgs` 已定义 |
| P1 | `GetRobotSnapshot` / `GetRobotFullInfo` / `GetActiveTask` / `GetCapabilities` | Unary 查询 |
| P1 | `EmergencyStop` / `CancelAllTasks` | 系统命令 |
| P2 | `SendFollowCommand` / `SendDockCommand` / `SendMapCommand` | 任务 Stream |
| P3 | `SendTeleopCommand` | **Bidi Stream**，需 async_grpc Bidi Handler |

(bridge-grpc-config)=
## 5.7 配置 → Server 参数映射

| `GrpcOptions` | `Server::Builder` | 当前 |
|---------------|-------------------|------|
| `host` + `port` | `SetServerAddress` | 硬编码 `127.0.0.1`，缺 port |
| `num_grpc_threads` | `SetNumGrpcThreads` | 4 |
| `num_event_threads` | `SetNumEventThreads` | 4 |
| — | `SetMaxSendMessageSize` | 100 MB |
| `enable_ssl_encryption` | 待封装 | 未实现 |
| `uplink_server_address` | `NavigatorStub` / Client | 空 |

**目标接线**：

```cpp
Server::Server(const proto::GrpcOptions& o) {
    Server::Builder b;
    b.SetServerAddress(o.host() + ":" + std::to_string(o.port()));
    b.SetNumGrpcThreads(o.num_grpc_threads());
    b.SetNumEventThreads(o.num_event_threads());
    // RegisterHandler…
    grpc_server_ = b.Build();
}
```

## 5.8 开发 checklist

- [x] Proto → `automsgs.rpcs.*` Handler（见 [grpc/07](07_handlers.md)）
- [x] `Context` = Muxer · Hub · DomainBundle · WorkScheduler
- [x] Command：ACK 优先 + `TaskMuxer` 互斥；CancelAll 经 Factory
- [ ] `GrpcOptions` 全量传入 Builder（host/port/threads）
- [ ] `grpcurl` / `rpc-cli.py` 端到端验证

细节：源码旁 `grpc/DESIGN.md`。

---

**导航**：[← 04 Handler API](04_handler_api.md) · [06 上游参考 →](06_upstream_reference.md)
