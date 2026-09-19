(rpc-system-api)=
# 系统控制 System

源文件：`automsgs/proto/rpcs/system.proto`  
Handler：`rpc_system_handlers`

全局 Unary，可抢占互斥任务。

## 6.1 方法

| RPC | 请求 | 响应 |
|-----|------|------|
| `EmergencyStop` | `EmergencyStopRequest`（`reason`） | `Status` |
| `ClearEmergencyStop` | `ClearEmergencyStopRequest` | `Status` |
| `CancelAllGoals` | `CancelAllGoalsRequest`（可选 `goal_kinds[]`） | `Status` |

另见查询类：`Heartbeat` / `GetInfo` / `GetStatus` / `GetHealth` / `GetRobotFullInfo` / `GetActiveGoal` / `GetCapabilities`（[04](04_query_api.md)）。

## 6.2 grpcurl

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/system.proto"
export SVC=automsgs.rpcs.system.SystemService

grpcurl -plaintext $PROTO_OPTS -d '{"reason":"operator"}' \
  "$BRIDGE" "$SVC/EmergencyStop"

grpcurl -plaintext $PROTO_OPTS -d '{}' \
  "$BRIDGE" "$SVC/ClearEmergencyStop"

grpcurl -plaintext $PROTO_OPTS -d '{}' \
  "$BRIDGE" "$SVC/CancelAllGoals"
```
