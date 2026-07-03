(bridge-grpc)=
# 3. gRPC 桥接插件

gRPC 插件是 Bridge **当前唯一可用**的传输实现（`plugins/grpc/`），基于 `async_grpc` 提供异步流式 RPC。

| 本文 §3 | 相关文档 |
|---------|----------|
| Handler、RPC、线程 | [§0 指南](00_guide.md) · [§2 架构](02_architecture.md) · [RPC 参考](rpcs/00_guide.md) |

---

## 3.1 组件概述

| 文件 | 类 | 职责 |
|------|-----|------|
| `grpc_bridge.hpp/cpp` | `GrpcBridgeServer` | 服务器生命周期、Handler 注册 |
| `grpc_bridge_server_interface.hpp` | `GrpcBridgeServerInterface` | 虚接口，便于测试替换 |
| `grpc_bridge_context.hpp` | `GrpcBridgeContextInterface` | RPC 执行上下文（待扩展） |
| `handlers/navigation_handler.*` | `SendNavigationHandler` | 导航指令 RPC |
| `handlers/exploration_handler.*` | `SendExplorationHandler` | 探索指令 RPC |
| `clients/navigator_stub.*` | — | Navigator 客户端（待实现） |

## 3.2 服务器构建

`GrpcBridgeServer` 构造函数使用 Builder 模式：

```cpp
autonomy::common::async_grpc::Server::Builder server_builder;
server_builder.SetServerAddress("127.0.0.1");
server_builder.SetNumGrpcThreads(4);
server_builder.SetNumEventThreads(4);
server_builder.SetMaxSendMessageSize(kMaxMessageSize);  // 100 MB

server_builder.RegisterHandler<handlers::SendNavigationHandler>();
server_builder.RegisterHandler<handlers::SendExplorationHandler>();
grpc_server_ = server_builder.Build();
```

### 3.2.1 Builder 参数对照

| Builder 方法 | Proto 字段 | 当前状态 |
|--------------|-----------|----------|
| `SetServerAddress` | `grpc.host` + `grpc.port` | 硬编码 |
| `SetNumGrpcThreads` | `grpc.num_grpc_threads` | 硬编码 4 |
| `SetNumEventThreads` | `grpc.num_event_threads` | 硬编码 4 |
| `SetMaxSendMessageSize` | — | 100 MB |
| `SetMaxReceiveMessageSize` | — | 默认 10 MB |

### 3.2.2 生命周期

```
构造 → Build() → Start() → [运行中] → Shutdown() → WaitForShutdown()
                      ↑                      │
                      └── task_thread_ ──────┘
```

| 方法 | 行为 |
|------|------|
| `Start()` | `shutting_down_=false`，启动 `task_thread_`，`grpc_server_->Start()` |
| `Shutdown()` | 置 `shutting_down_=true`，关闭 gRPC，join 后台线程 |
| `WaitForShutdown()` | 阻塞至 gRPC 完全退出 |
| `WaitUntilIdle()` | 测试用，当前为空实现 |

## 3.3 async_grpc 框架

`async_grpc` 采用 **Completion Queue（CQ）驱动**的异步模型：

1. Server 在 CQ 上注册 RPC 请求
2. 事件到达时，Event 线程调度对应 `RpcHandler`
3. Handler 通过 `Send()` / `Finish()` 完成响应

Handler 基类关键接口：

```cpp
template <typename Signature>
class RpcHandler {
    virtual void OnRequest(const IncomingType& request);
    virtual void OnReadsDone();
    void Send(std::unique_ptr<OutgoingType> message);
    void Finish(const ::grpc::Status& status);
    ExecutionContext* GetContext();
};
```

### 3.3.1 线程安全

- `GetContext()`：同步上下文，Handler 回调内安全访问
- `GetUnsynchronizedContext()`：仅在同一线程内使用
- 跨 Handler 共享状态应通过 `GrpcBridgeContextInterface` 注入

## 3.4 AutonomyService 接口

完整 RPC 字段、枚举与 `grpcurl` / Python 示例见 **[RPC · AutonomyService](rpcs/01_autonomy_service.md)**。以下为 Handler 侧要点摘要。

### 3.4.1 SendNavigationCommand

Handler：`SendNavigationHandler`。预期在 `OnRequest` 内按 `command` 分支，对 `NAV_CMD_START` 做 TF 变换并调用 Navigator，期间多次 `Send()`。

### 3.4.2 SendExplorationCommand

Handler：`SendExplorationHandler`。`oneof params` 在 `SET_AREA` / `SAVE_MAP` 时分别使用 `Polygon` / `map_name`。

### 3.4.3 ReceiveBotStates / ReceiveBotEvents

Proto 已定义，Handler **未注册**；`RobotState` / `RobotEvent` 字段待在 commmsgs 中补充。

## 3.5 自定义 Handler

添加新 RPC 的标准流程：

**Step 1** — 在 proto 中声明：

```protobuf
rpc MyCustomCommand(MyRequest) returns (stream MyResponse);
```

**Step 2** — 定义 Handler：

```cpp
DEFINE_HANDLER_SIGNATURE(
    MyCustomSignature, proto::MyRequest,
    async_grpc::Stream<proto::MyResponse>,
    "/autonomy.bridge.proto.AutonomyService/MyCustomCommand")

class MyCustomHandler
    : public async_grpc::RpcHandler<MyCustomSignature> {
public:
    void OnRequest(const proto::MyRequest& request) override {
        auto resp = std::make_unique<proto::MyResponse>();
        resp->set_success(true);
        Send(std::move(resp));
        Finish(::grpc::Status::OK);
    }
};
```

**Step 3** — 注册：

```cpp
server_builder.RegisterHandler<MyCustomHandler>();
```

## 3.6 后台任务线程

`ProcessSensorDataQueue()` 当前为占位实现：

```cpp
while (!shutting_down_) {
    // 预留给传感器数据上行队列消费
    std::this_thread::sleep_for(std::chrono::seconds(5));
}
```

规划用途：

- 批量聚合 `RobotState` 推送至 `uplink_server_address`
- 异步处理大载荷（点云、地图切片）避免阻塞 gRPC 线程

队列弹出超时常量：`kPopTimeout = 100 ms`。

## 3.7 gRPC 状态码约定

| gRPC Status | 场景 |
|-------------|------|
| `OK` | 指令正常处理完毕 |
| `INVALID_ARGUMENT` | 枚举非法、poses 为空、多边形退化 |
| `NOT_FOUND` | 引用的轨迹/地图 ID 不存在 |
| `FAILED_PRECONDITION` | FSM 状态不允许该指令 |
| `UNAVAILABLE` | Navigator 未就绪 |
| `DEADLINE_EXCEEDED` | 客户端超时 |

## 3.8 性能调优

| 参数 | 建议 | 理由 |
|------|------|------|
| `num_grpc_threads` | CPU 核数 / 2 | I/O 密集 |
| `num_event_threads` | ≥ `num_grpc_threads` | Handler 可能阻塞 |
| `max_send_message_size` | 按需，默认 100 MB | 大地图/路径序列 |
| HTTP/2 keepalive | 启用 | 长连接 Stream 保活 |

并发 RPC 数近似：

$$
N_{\text{active}} \leq N_{\text{grpc}} \times \eta, \quad \eta \approx 2\text{-}4
$$

其中 $\eta$ 为每线程可重叠的 CQ 事件数。

## 3.9 与 Navigator 对接（规划）

`NavigatorStub` 将封装对 Navigator Action 的 gRPC/进程内调用：

```
SendNavigationHandler
        │
        ▼
NavigatorStub::SendGoal(poses)
        │
        ▼
Navigator BT → compute_path_to_pose
        │
        ▼
回调 → Handler::Send(progress_response)
```

关键映射：

| Bridge 命令 | Navigator 动作 |
|-------------|----------------|
| `NAV_CMD_START` | `NavigateToPose` / 多点巡航 |
| `NAV_CMD_PAUSE` | BT 暂停节点 |
| `NAV_CMD_RESUME` | BT 恢复节点 |
| `NAV_CMD_CANCEL` | `cancel_goal` |

---

**导航**：[← §2 架构](02_architecture.md) · [§4 MQTT →](04_mqtt.md)
