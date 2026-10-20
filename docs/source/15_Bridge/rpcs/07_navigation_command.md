(rpc-navigation-command)=
# NavigationService

源：`navigation.proto`。Handler → `NavigatorStub`。

| RPC | 模式 | 说明 |
|-----|------|------|
| `Navigate` | Unary→Stream | waypoints 1=单点，N=多点 |
| `Pause` / `Resume` / `Replan` / `Cancel` | Unary | 空 goal_id=当前 |
| `GetStatus` | Unary | 与流帧同形 |

要点：`goal_id` 幂等；状态含 IDLE/PLANNING/RUNNING/PAUSED/ARRIVED/FAILED/CANCELLED；忙 → `NAVIGATION_BUSY`。

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/navigation.proto"
export SVC=automsgs.rpcs.navigation.NavigationService
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetStatus"
```
