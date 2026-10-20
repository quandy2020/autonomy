(rpc-query-api)=
# 查询接口

**源文件：** `automsgs/proto/rpcs/system.proto`（系统面）+ 各域 `GetStatus`（见 Command 页）。

在发 Navigate / Follow 等 **Command 之前**，或轮询机载状态、不做长 stream 时

**和 Stream 的分工**

| 需求 | 用 Query | 用 Stream |
|------|----------|-----------|
| 导航进度 | `NavigationService/GetStatus` 快照 | `Navigate` stream |
| 全局事件 | （无统一 BotEvents） | 各 Command stream |
| 能力/空闲 | `GetCapabilities` / `GetActiveGoal` | 用 Unary 即可，不必挂 Stream |

历史上全局 `ReceiveBotStates/Events` 已移除；不要在新客户端里找统一 push stream。

### SystemService（Unary）

| RPC | 用途 | 备注 |
|-----|------|------|
| `Heartbeat` | 连通 + 时钟 | 轻量保活 |
| `GetInfo` | 型号、版本、hostname | 静态偏多 |
| `GetStatus` | 粗状态（含 `SYSTEM_STATE_BUSY` 等） | Muxer 有活动任务时可能 BUSY |
| `GetHealth` | CPU/内存/通道/watch | 对接 system monitor |
| `GetRobotFullInfo` | 身份 + 状态 + 能力一次打包 | 云侧 onboarding 常用 |
| `GetActiveGoal` | 当前互斥任务类型与 goal_id | 发新 Command 前建议先看 |
| `GetCapabilities` | `supports_*` + bridge 版本 | **联调第一步** |

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/system.proto"
export SVC=automsgs.rpcs.system.SystemService
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetCapabilities"
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetActiveGoal"
```

### 常见坑

| 现象 | 说明 |
|------|------|
| `supports_navigation=false` | Capabilities 配置关闭了宣称；或旧 conf |
| `GetStatus` 一直 BUSY | 有未结束 Command；查 `GetActiveGoal` 再 Cancel |
| 与 Navigate stream 不一致 | 快照落后一帧正常；以 stream 为准做 UI 进度 |

系统控制（Estop/CancelAll）见 [06 系统控制](06_system_api.md)。
