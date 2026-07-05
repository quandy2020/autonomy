(rpc-system-api)=
# 系统控制 System

全局 Unary RPC，可抢占任意活跃 Command 任务。用于**安全测试**与**异常恢复**。

## 6.1 方法列表

| RPC | 请求 | 响应 |
|-----|------|------|
| `EmergencyStop` | `EmergencyStopRequest` | `CommandAck` |
| `CancelAllTasks` | `CancelAllTasksRequest` | `CommandAck` |

Unary 直接返回 `CommandAck`（非 Stream，无 `final` 语义）。

## 6.2 请求字段

**EmergencyStopRequest**：`header` + `reason`（string）

**CancelAllTasksRequest**：`header` + `task_types[]`（**空数组 = 取消全部**）

## 6.3 grpcurl

JSON 样例 → [01 §1.4](01_connection_guide.md#14-响应-json-参考) · 完整用例 → [13 §13.5](13_integration_tests.md#135-system-测试用例)。

**EmergencyStop**

```bash
# EmergencyStop — 全局急停（System · Unary · 最高优先级）
export CMD_ID=$(new_cmd_id)

grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/EmergencyStop <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "reason": "operator"
}
EOF
```


**CancelAllTasks — 全部**

```bash
# CancelAllTasks — 取消全部活跃任务（task_types 省略 = 全部）
export CMD_ID=$(new_cmd_id)

grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/CancelAllTasks <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  }
}
EOF
```

**CancelAllTasks — 只取消导航(1)和探索(4)**

```bash
# CancelAllTasks — 按 TaskType 选择性取消（1=导航, 4=探索）
export CMD_ID=$(new_cmd_id)

grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/CancelAllTasks <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "task_types": [
    1,
    4
  ]
}
EOF
```


MQTT：`cmd/emergency_stop`（`EmergencyStop`）。
