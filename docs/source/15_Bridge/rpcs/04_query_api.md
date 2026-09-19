(rpc-query-api)=
# 查询接口 Query

无副作用只读 RPC，位于 **`SystemService`**（及 Loc / Sensor / Map 资源 RPC）。调用命令前建议先 `GetCapabilities`。

源文件：`automsgs/proto/rpcs/system.proto`

## 4.1 System 查询

| RPC | 响应 | 用途 |
|-----|------|------|
| `Heartbeat` | `HeartbeatResponse` | 连通 / 时钟 |
| `GetInfo` | `GetInfoResponse` | 型号、序列号、版本 |
| `GetStatus` | `GetStatusResponse` | 粗状态 + 电池 + 健康摘要 |
| `GetHealth` | `GetHealthResponse` | 完整通道 / 延迟健康 |
| `GetRobotFullInfo` | `RobotFullInfo` | 身份 + `RobotState` + 活跃目标 + 能力 |
| `GetActiveGoal` | `ActiveGoal` | 当前互斥任务（`kind` + `goal_id`） |
| `GetCapabilities` | `Capabilities` | `supports_*` + 版本 |

历史 `GetRobotSnapshot` / `GetActiveTask` / `ReceiveBot*` 已移除；状态走 `GetStatus` / `GetRobotFullInfo` / `GetActiveGoal`。

## 4.2 Capabilities（摘要）

| 字段 | 说明 |
|------|------|
| `supports_navigation` / `follow` / `charge` / `mapping` / `teleop` / … | 能力门控 |
| `bridge_version` / `autonomy_version` | 版本字符串 |

对照 [02 §2.5](02_service_overview.md#25-command-与-tasktype-对照)。

## 4.3 grpcurl

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/system.proto"
export SVC=automsgs.rpcs.system.SystemService
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetCapabilities"
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetActiveGoal"
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetRobotFullInfo"
```
