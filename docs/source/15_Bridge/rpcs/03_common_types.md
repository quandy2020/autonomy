(rpc-common-types)=
# 公共消息类型

定义：`automsgs/proto/rpcs/common.proto`。

### 3.1 Status

| 字段 | 含义 |
|------|------|
| `code` | `StatusCode`（成功=`OK`） |
| `message` | 可读说明 |

### 3.2 GoalRequest

```protobuf
message GoalRequest { string goal_id = 1; } // 空 = 当前目标
```

Pause / Resume / Cancel 共用。

### 3.3 TaskType

| 值 | 枚举 | 服务 |
|----|------|------|
| 1–6 | NAVIGATION … MAP | 见 [02 §2.3](02_service_overview.md) |

### 3.4 命令流时序

```text
Client → Navigate/… → 接受帧 → 进度帧* → 终态/Cancel
```

细节以各域 proto 为准。
