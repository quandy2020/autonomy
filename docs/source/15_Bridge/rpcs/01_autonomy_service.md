(bridge-autonomy-service)=
# 1. AutonomyService

`external_command_service.proto` 中的对外 gRPC 服务。包名 **`autonomy.bridge.proto`**，服务名 **`AutonomyService`**。

| 本文 §1 | 相关文档 |
|---------|----------|
| RPC 字段与调用示例 | [§0 RPC 指南](00_guide.md) · [§3 gRPC Handler](../03_grpc.md) · [commsgs geometry_msgs](../../14_Commsgs/06_core_msgs.md) |

---

## 1.1 服务定义

```protobuf
service AutonomyService {
  rpc ReceiveBotStates(google.protobuf.Empty)
      returns (stream commsgs.proto.vehicle_msgs.RobotState);
  rpc ReceiveBotEvents(google.protobuf.Empty)
      returns (stream commsgs.proto.vehicle_msgs.RobotEvent);
  rpc SendNavigationCommand(NavigationCommandRequest)
      returns (stream NavigationCommandResponse);
  rpc SendExplorationCommand(ExplorationCommandRequest)
      returns (stream ExplorationCommandResponse);
}
```

| 方法路径 | 流模式 |
|----------|--------|
| `/autonomy.bridge.proto.AutonomyService/SendNavigationCommand` | Unary-Stream |
| `/autonomy.bridge.proto.AutonomyService/SendExplorationCommand` | Unary-Stream |
| `/autonomy.bridge.proto.AutonomyService/ReceiveBotStates` | Unary-Stream |
| `/autonomy.bridge.proto.AutonomyService/ReceiveBotEvents` | Unary-Stream |

---

## 1.2 SendNavigationCommand

**签名**

```protobuf
rpc SendNavigationCommand(NavigationCommandRequest)
    returns (stream NavigationCommandResponse);
```

**Handler**：`SendNavigationHandler`（`navigation_handler.cpp`，当前 `OnRequest` 为空实现）

### 1.2.1 NavigationCommandRequest

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `timestamp` | `builtin_interfaces.Time` | 建议 | 客户端发送时刻；服务端可拒绝过期请求 |
| `command` | `NavigationCommand` | 是 | 指令枚举 |
| `poses` | `geometry_msgs.PoseArray` | `START` 时 | 目标位姿序列；`header.frame_id` 为位姿参考系 |

**NavigationCommand**

| 值 | 名称 | 含义 | `poses` |
|----|------|------|---------|
| 0 | `NAV_CMD_UNSPECIFIED` | 无效 | — |
| 1 | `NAV_CMD_START` | 开始导航 | 必填 |
| 2 | `NAV_CMD_STOP` | 停止 | 忽略 |
| 3 | `NAV_CMD_CANCEL` | 取消当前目标 | 忽略 |
| 4 | `NAV_CMD_RESUME` | 恢复暂停 | 忽略 |
| 5 | `NAV_CMD_PAUSE` | 暂停 | 忽略 |

**预期服务端行为**（规划）：

1. 校验 FSM：当前 Navigator 状态是否允许该 `command`  
2. `NAV_CMD_START`：将 `poses` 变换到全局系，调用 Navigator Action  
3. 执行过程中多次 `Send(NavigationCommandResponse)` 推送进度  
4. 结束或失败时 `Finish(grpc::Status)`  

**与 Navigator 映射**（规划）：

| `command` | 内部动作 |
|-----------|----------|
| `NAV_CMD_START` | `NavigateToPose` / 多点巡航 |
| `NAV_CMD_PAUSE` | BT 暂停 |
| `NAV_CMD_RESUME` | BT 恢复 |
| `NAV_CMD_CANCEL` / `NAV_CMD_STOP` | `cancel_goal` |

### 1.2.2 NavigationCommandResponse

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 本条流消息是否表示成功状态 |
| `message` | `string` | 人类可读说明或错误摘要 |

流在 RPC 存活期间可推送**多条**响应；客户端应迭代读取直至流结束。

### 1.2.3 调用示例

**grpcurl — 停止导航**

```bash
grpcurl -plaintext -d '{"command": 2}' 127.0.0.1:5005 \
  autonomy.bridge.proto.AutonomyService/SendNavigationCommand
```

**grpcurl — 单点导航**（`frame_id` 与位姿按现场 TF 填写）

```bash
grpcurl -plaintext -d '{
  "timestamp": {"sec": 0, "nanosec": 0},
  "command": 1,
  "poses": {
    "header": {"frame_id": "map"},
    "poses": [{"position": {"x": 1.0, "y": 2.0, "z": 0.0}}]
  }
}' 127.0.0.1:5005 \
  autonomy.bridge.proto.AutonomyService/SendNavigationCommand
```

---

## 1.3 SendExplorationCommand

**签名**

```protobuf
rpc SendExplorationCommand(ExplorationCommandRequest)
    returns (stream ExplorationCommandResponse);
```

**Handler**：`SendExplorationHandler`（业务待实现）

### 1.3.1 ExplorationCommandRequest

| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | `Time` | 同导航请求 |
| `command` | `ExplorationCommand` | 探索指令 |
| `params` | `oneof` | 按指令选用 `area` 或 `map_name` |

**ExplorationCommand**

| 值 | 名称 | 含义 |
|----|------|------|
| 0 | `EXPLORATION_CMD_UNSPECIFIED` | 无效 |
| 1 | `EXPLORATION_CMD_START` | 开始探索 |
| 2 | `EXPLORATION_CMD_STOP` | 停止 |
| 3 | `EXPLORATION_CMD_PAUSE` | 暂停 |
| 4 | `EXPLORATION_CMD_RESUME` | 恢复 |
| 5 | `EXPLORATION_CMD_CANCEL` | 取消 |
| 6 | `EXPLORATION_CMD_SAVE_MAP` | 保存地图，需 `map_name` |
| 7 | `EXPLORATION_CMD_SET_AREA` | 设置区域，需 `area` |

**oneof params**

| 指令 | 字段 | 类型 |
|------|------|------|
| `EXPLORATION_CMD_SET_AREA` | `area` | `geometry_msgs.Polygon` |
| `EXPLORATION_CMD_SAVE_MAP` | `map_name` | `string` |

### 1.3.2 ExplorationCommandResponse

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 本条是否成功 |
| `message` | `string` | 说明或错误 |
| `status` | `ExplorationStatus` | 当前探索状态 |

**ExplorationStatus**

| 值 | 名称 |
|----|------|
| 0 | `EXPLORATION_STATUS_UNKNOWN` |
| 1 | `EXPLORATION_STATUS_IDLE` |
| 2 | `EXPLORATION_STATUS_EXPLORING` |
| 3 | `EXPLORATION_STATUS_PAUSED` |
| 4 | `EXPLORATION_STATUS_COMPLETED` |

### 1.3.3 调用示例

**grpcurl — 开始探索**

```bash
grpcurl -plaintext -d '{"command": 1}' 127.0.0.1:5005 \
  autonomy.bridge.proto.AutonomyService/SendExplorationCommand
```

**grpcurl — 保存地图**

```bash
grpcurl -plaintext -d '{
  "command": 6,
  "map_name": "warehouse_floor1"
}' 127.0.0.1:5005 \
  autonomy.bridge.proto.AutonomyService/SendExplorationCommand
```

---

## 1.4 ReceiveBotStates / ReceiveBotEvents

**签名**

```protobuf
rpc ReceiveBotStates(google.protobuf.Empty)
    returns (stream commsgs.proto.vehicle_msgs.RobotState);
rpc ReceiveBotEvents(google.protobuf.Empty)
    returns (stream commsgs.proto.vehicle_msgs.RobotEvent);
```

| RPC | 用途 | Handler | 状态 |
|-----|------|---------|------|
| `ReceiveBotStates` | 周期推送机器人状态 | 未实现 | ❌ |
| `ReceiveBotEvents` | 推送事件（故障、任务切换等） | 未实现 | ❌ |

> `vehicle_msgs.RobotState` / `RobotEvent` 在 `vehicle_msgs.proto` 中**尚无字段**，需先扩展 commmsgs 再实现 Bridge 订阅与填充。

**预期客户端用法**：建立长连接后循环 `Read()`，直至 `Cancel` 或连接断开；Bridge 在 `OnCancel` 中清理内部订阅。

**grpcurl — 拉取状态流**（需 proto 描述；输出为连续 JSON 行）

```bash
grpcurl -plaintext -d '{}' 127.0.0.1:5005 \
  autonomy.bridge.proto.AutonomyService/ReceiveBotStates
```

---

## 1.5 客户端示例（Python）

```python
import grpc
from autonomy.bridge.proto import external_command_service_pb2 as pb
from autonomy.bridge.proto import external_command_service_pb2_grpc as stub_mod

CHANNEL = "127.0.0.1:5005"
stub = stub_mod.AutonomyServiceStub(grpc.insecure_channel(CHANNEL))

# 导航：停止
req = pb.NavigationCommandRequest(command=pb.NAV_CMD_STOP)
for resp in stub.SendNavigationCommand(req):
    print("nav:", resp.success, resp.message)

# 探索：开始
expl_req = pb.ExplorationCommandRequest(command=pb.EXPLORATION_CMD_START)
for resp in stub.SendExplorationCommand(expl_req):
    print("explore:", resp.success, resp.status, resp.message)

# 状态流（Handler 未实现时可能无数据或立即结束）
# for state in stub.ReceiveBotStates(google.protobuf.empty_pb2.Empty()):
#     print(state)
```

生产环境将 `insecure_channel` 换为 `secure_channel` 并配置凭证。

---

## 1.6 gRPC 状态码（约定）

Handler 实现时应与 [§3.7](../03_grpc.md#37-grpc-状态码约定) 一致：

| 码 | 场景 |
|----|------|
| `OK` | 正常结束流 |
| `INVALID_ARGUMENT` | 枚举非法、缺少 `poses` |
| `FAILED_PRECONDITION` | FSM 不允许该指令 |
| `NOT_FOUND` | 无活动导航/探索会话 |
| `UNAVAILABLE` | Navigator 未就绪 |

---

**导航**：[← §0 RPC 指南](00_guide.md) · [§2 BridgeOptions →](02_bridge_options.md)
