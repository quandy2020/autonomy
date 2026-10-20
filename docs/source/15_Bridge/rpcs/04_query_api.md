(rpc-query-api)=
# 查询接口

源：`system.proto`。调用命令前建议 `GetCapabilities`。

| RPC | 用途 |
|-----|------|
| `Heartbeat` | 连通 |
| `GetInfo` / `GetStatus` / `GetHealth` | 身份 / 粗状态 / 健康 |
| `GetRobotFullInfo` | 身份+状态+能力 |
| `GetActiveGoal` | 当前互斥任务 |
| `GetCapabilities` | `supports_*` + 版本 |

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/system.proto"
export SVC=automsgs.rpcs.system.SystemService
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetCapabilities"
```
