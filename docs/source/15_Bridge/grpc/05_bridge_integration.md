(async-grpc-bridge-integration)=
# Bridge 集成

> [§4 gRPC 总览](../04_grpc.md) · 本专题 **grpc/05**（H2 **5.x**）。

`autonomy/bridge` 源码链路：**配置 → BridgeServer → GrpcBridgeServer → async_grpc → Handler → Proto**。

---

## 5.1 集成总览

```
bridge.lua / BridgeOptions
    → BridgeServer → GrpcBridgeServer
        → Server::Builder → Server (AutonomyService)
            → SendNavigationHandler / SendExplorationHandler
```

框架背景：[cartographer async_grpc](https://github.com/cartographer-project/async_grpc) · 类图：[grpc/01](01_overview.md)。

## 5.2 启动链路

**BridgeServer**（`bridge_server.cpp`）：

```cpp
BridgeServer::BridgeServer() {
    grpc_bridge_ = std::make_unique<GrpcBridgeServer>();
}
void BridgeServer::Start() {
    grpc_bridge_->Start();  // 当前无论 use_grpc 均启动
}
```

**GrpcBridgeServer** 构造（`grpc_bridge.cpp`）：

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

## 5.5 ExecutionContext（规划）

`GrpcBridgeContextInterface` 继承 `ExecutionContext`，规划注入 `NavigatorStub`、TF、FSM。须在 `Build()` 后、`Start()` 前 `SetExecutionContext`。

## 5.6 待实现 RPC

按 [grpc/07_handlers](../grpc/07_handlers.md#71-handler-对照表) 优先级：

| 优先级 | RPC | 说明 |
|--------|-----|------|
| P0 | `ReceiveBotStates` / `ReceiveBotEvents` | Empty → Stream；`vehicle_msgs` 已定义 |
| P1 | `GetRobotSnapshot` / `GetActiveTask` / `GetCapabilities` | Unary 查询 |
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
GrpcBridgeServer::GrpcBridgeServer(const proto::GrpcOptions& o) {
    Server::Builder b;
    b.SetServerAddress(o.host() + ":" + std::to_string(o.port()));
    b.SetNumGrpcThreads(o.num_grpc_threads());
    b.SetNumEventThreads(o.num_event_threads());
    // RegisterHandler…
    grpc_server_ = b.Build();
}
```

## 5.8 开发 checklist

- [ ] Proto 声明 RPC → 生成 `.pb.h`（当前 13 RPC 已定义）
- [ ] 按 [grpc/07 Handler 规划](../grpc/07_handlers.md#71-handler-对照表) 逐类 `RegisterHandler`
- [ ] `DEFINE_HANDLER_SIGNATURE` 全名与 proto 一致
- [ ] Command：`header` 校验 + `CommandAck` 流（`final` 末帧）
- [ ] Push：`ReceiveBotStates` 填充 [vehicle_msgs](../rpcs/05_stream_api.md#53-vehicle_msgs)
- [ ] `SetExecutionContext` + FSM / `NavigatorMuxer`
- [ ] `GrpcOptions` 传入 Builder（P0）
- [ ] `grpcurl` / Python 客户端验证

---

**导航**：[← 04 Handler API](04_handler_api.md) · [06 上游参考 →](06_upstream_reference.md)
