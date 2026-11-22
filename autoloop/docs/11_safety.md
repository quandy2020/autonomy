# 11 安全域

L4 只闩锁：置位后保持到人工 `Clear()`。不改写 `/cmd_vel`，采集进程不插拦截器。编译期 `AUTOLOOP_LATCH_ONLY=1`（`cmake/SafetyChecks.cmake`）。制动由 `autonomy.monitor` MRM 读闩执行。

## 进程与文件

| 角色 | 动作 |
|------|------|
| `autoloop.recorder safety` / `safety_guard` | 订阅 cmd / teleop；超限 `SetLatched`；追加哈希链 |
| `SafetyLatch` 文件 | 进程间闩 |
| `autonomy.monitor` | 读闩 → 停机 |
| Bridge E-Stop | 写同一闩文件 |

路径：`AUTOLOOP_SAFETY_LATCH_PATH` → `AUTONOMY_SAFETY_LATCH_PATH` → `~/.autonomy/runtime/safety_latch`。

## 裁决

`EvaluateCommand`：

| 条件 | 结果 |
|------|------|
| 已闩 | Pass（不再重复置位） |
| teleop | Handover，只记账 |
| `\|vx\|>max_cmd_vx` 或 `\|wz\|>max_cmd_wz` | Fallback → `SetLatched(true, "safety_monitor_cmd_limit")` |

不向 `/cmd_vel` 写零速。HIL：闩已置则拒绝回放灌总线。

## 哈希链

安全事件追加 `prev_hash → entry_hash`（SHA-256，`hash_util`），持久化 `safety/chain.bin`。`verify-chain` / 云端 Verify 失败 → 告警 `safety_event`，该 robot 制品不得晋升。链损坏不自动截断，需运维介入。

P0 急停 / 任务失败在 recorder flush 中追加链项。
