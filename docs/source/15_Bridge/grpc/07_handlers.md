(grpc-handlers)=
# Handler 注册与规划

> [§4 gRPC 总览](../04_grpc.md) · **grpc/07** · H2 **7.x**。

**13 RPC Handler 实现状态的单一维护点**（⏳ / ❌）。契约见 [rpcs/02 服务概述](../rpcs/02_service_overview.md) · 接线见 [05 Bridge 集成](05_bridge_integration.md)。

## 7.1 Handler 对照表

| RPC | 规划类名 | 流模式 | 状态 |
|-----|----------|--------|------|
| `SendNavigationCommand` | `SendNavigationHandler` | Unary→Stream | ⏳ 已注册，`OnRequest` 空 |
| `SendExplorationCommand` | `SendExplorationHandler` | Unary→Stream | ⏳ 已注册，业务空 |
| `SendFollowCommand` | `SendFollowHandler` | Unary→Stream | ❌ |
| `SendTeleopCommand` | `SendTeleopHandler` | **Bidi** | ❌ |
| `SendDockCommand` | `SendDockHandler` | Unary→Stream | ❌ |
| `SendMapCommand` | `SendMapHandler` | Unary→Stream | ❌ |
| `ReceiveBotStates` | `ReceiveBotStatesHandler` | Empty→Stream | ❌ |
| `ReceiveBotEvents` | `ReceiveBotEventsHandler` | Empty→Stream | ❌ |
| `GetRobotSnapshot` | `GetRobotSnapshotHandler` | Unary | ❌ |
| `GetActiveTask` | `GetActiveTaskHandler` | Unary | ❌ |
| `GetCapabilities` | `GetCapabilitiesHandler` | Unary | ❌ |
| `EmergencyStop` | `EmergencyStopHandler` | Unary | ❌ |
| `CancelAllTasks` | `CancelAllTasksHandler` | Unary | ❌ |

(push-handler-signature)=
## 7.2 Push Handler 签名

```cpp
DEFINE_HANDLER_SIGNATURE(
    BotStatesSignature,
    google::protobuf::Empty,
    autonomy::common::async_grpc::Stream<
        autonomy::commsgs::proto::vehicle_msgs::RobotState>,
    "/autonomy.bridge.proto.AutonomyService/ReceiveBotStates")
```

## 7.3 FSM 与 Stream 约定

`CommandFsm` / `NavigatorMuxer` 保证任务互斥；`EmergencyStop` 可抢占任意任务。时序见 [rpcs/03 §3.5](../rpcs/03_common_types.md#35-command-stream-时序)。

Command Handler：

1. 校验 `header`（`cmd_id` 去重、`timestamp` 窗口、`nonce`）
2. 首帧 `Send()`：`ack.success=true`，`final=false`
3. 进度帧：同一 `cmd_id`，`final=false`
4. 末帧：`ack.final=true`，再 `Finish(OK)`

Teleop Bidi：`OnRequest` 处理速度流；`watchdog_timeout_sec` 超时发 `TELEOP_CMD_STOP`。

## 7.4 gRPC 状态码

与 [§4.4](../04_grpc.md#44-状态码与调优) 及 `CommandAck.error_code`（[Commsgs error_code](../../14_Commsgs/08_nav_planning_msgs.md#83-error_code)）一致：

| 码 | 场景 |
|----|------|
| `OK` | 正常结束 |
| `INVALID_ARGUMENT` | 枚举非法、缺少 `goals` / `header` |
| `FAILED_PRECONDITION` | FSM 不允许、任务互斥 |
| `NOT_FOUND` | 无活跃会话 |
| `UNAVAILABLE` | Navigator 未就绪 |

---

**导航**：[← 06 上游参考](06_upstream_reference.md) · [§4 gRPC 总览 →](../04_grpc.md)
