(rpc-dock-command)=
# ChargeService

**源文件：** `automsgs/proto/rpcs/charge.proto`。

在回充桩、离桩；文档与 proto 已用 **Charge** 命名（旧「Dock」页指同一服务）

### RPC

| RPC | 模式 |
|-----|------|
| `Return` | Unary → Stream（回桩） |
| `Leave` | Unary → Stream（离桩） |
| Pause / Resume / Cancel / GetStatus | Unary |

`station_id` 空表示默认桩（实现定义，见 conf）。

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/charge.proto"
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" automsgs.rpcs.charge.ChargeService/GetStatus
```

### 互斥

进行中的导航/跟随/遥操时发起 Return → `CHARGING_BUSY`（或同等 Status）。Capabilities 里 `supports_charge` / docking 相关位需为 true。

### Stream 注意

Return/Leave 进度在 stream；Unary Cancel 后应读 stream 终态或 `GetStatus` 确认 IDLE。

### 常见坑

| 现象 | 说明 |
|------|------|
| 回桩不动 | 定位/桩位/对接动作链未起；非 Bridge  alone |
| 与 Navigate 并发 | 应 busy；先 Cancel |
| Capabilities 无 charge | [01 参数](../01_options.md) `CapabilitiesOptions` |
