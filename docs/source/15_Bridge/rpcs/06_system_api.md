(rpc-system-api)=
# 系统控制

**源文件：** `automsgs/proto/rpcs/system.proto`。全部为 **Unary**，可抢占进行中的 Command Stream。

在现场急停、清急停、或不管当前在跑什么任务要 **一键清空互斥槽**

**与 Command 的关系**

| 操作 | 对 Navigate stream 的影响 |
|------|---------------------------|
| Estop | 应出现 CANCELED/FAILED 类终态或拒绝后续命令 |
| CancelAll | 清 muxer；之后可新发 Navigate |
| ClearEstop | 不自动恢复旧 Goal；通常需重新 Navigate |

Estop 期间再发 Navigate/Follow 可能被 gate（视 Handler 与 `session` 状态）。

### RPC

| RPC | 用途 |
|-----|------|
| `EmergencyStop` | 急停；带 `reason` 便于日志 |
| `ClearEmergencyStop` | 清除急停（仍需满足安全前置） |
| `CancelAllGoals` | 取消活跃目标；可选按 `goal_kinds` 过滤 |

查询类（Capabilities、ActiveGoal、Health）见 [04 查询](04_query_api.md)。

```bash
grpcurl -plaintext $PROTO_OPTS -d '{"reason":"operator"}' "$BRIDGE" "$SVC/EmergencyStop"
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/ClearEmergencyStop"
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/CancelAllGoals"
```

### 常见坑

| 现象 | 先查 |
|------|------|
| Clear 后仍不能导航 | 是否仍 Estop；planning/control 是否存活 |
| CancelAll 后 GetActiveGoal 非空 | 域 stub 异步收尾；稍后再查或看 task 日志 |
| 只调 Estop 不调 Clear | 后续 Command 一直被拒 |

管理面还有 `RestartModule` / OTA 等 RPC，面向运维；日常联调以上三个足够。
