(rpc-system-api)=
# 系统控制

源：`system.proto`。全局 Unary，可抢占。

| RPC | 说明 |
|-----|------|
| `EmergencyStop` | `reason` |
| `ClearEmergencyStop` | 清除 |
| `CancelAllGoals` | 可选 `goal_kinds[]` |

查询类见 [04](04_query_api.md)。

```bash
grpcurl -plaintext $PROTO_OPTS -d '{"reason":"operator"}' "$BRIDGE" "$SVC/EmergencyStop"
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/ClearEmergencyStop"
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/CancelAllGoals"
```
