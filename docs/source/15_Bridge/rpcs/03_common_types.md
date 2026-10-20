(rpc-common-types)=
# 公共消息类型

**定义：** `automsgs/proto/rpcs/common.proto` + `automsgs/msgs/status_msgs`（统一 StatusCode）。

**什么时候看：** 解析拒绝帧、对接自家 Status 枚举、或写 Cancel/Pause 载荷时。

### Status

| 字段 | 含义 |
|------|------|
| `code` | `StatusCode`；成功为 `OK`（0） |
| `message` | 人可读原因（busy、estop、参数错等） |

业务 busy 多在 **响应 Status** 里，而不一定是 gRPC `UNAVAILABLE`。

### GoalRequest

```protobuf
message GoalRequest { string goal_id = 1; } // 空 = 当前目标
```

Pause / Resume / Replan / Cancel 共用。空 `goal_id` 表示「当前 muxer 上的目标」。

### 命令流（概念）

```text
Client → Navigate/… → （接受或拒绝帧）→ 进度帧* → FAILED/CANCELLED/断流
导航 ARRIVED 时 stream 可能仍保持 → 见 navigation.proto 注释
```

### 常见坑

| 坑 | 说明 |
|----|------|
| 只认 gRPC OK | 首帧 `status.code` 仍可能是 BUSY |
| goal_id 不一致 | Cancel 用了旧 id → 无效果 |
| 混用 TaskType 与 service 名 | 以 `GetActiveGoal` + proto 为准 |

TaskType 表见 [02 §2.3](02_service_overview.md)。
