(rpc-dock-command)=
# ChargeService

源：`charge.proto`（替代旧 Dock 命令）。

| RPC | 模式 |
|-----|------|
| `Return` / `Leave` | Unary→Stream |
| Pause/Resume/Cancel/GetStatus | Unary |

`station_id` 空=默认桩。忙 → `CHARGING_BUSY`。

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/charge.proto"
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" automsgs.rpcs.charge.ChargeService/GetStatus
```
