# 08 回放（可复现）

复现目标：同一 `VersionTuple` 下，把 MCAP 再喂进仿真或实机夹具，得到可对比的轨迹 / 任务结果。回放不是在线控制。

## 剖面

| Profile | 输入 | 跳过 | 用途 |
|---------|------|------|------|
| `open_loop` | 记录的 `/cmd_vel` | 无 | 执行器跟踪；仅离线、无活 controller |
| `mid_stack` | 感知 / 定位输出 | `cmd_vel*` | 默认 SIL：规划+控制回归 |
| `sensor_in` | lidar / camera / imu | 再跳过 `/plan`、BT、task、`/perception/*` | 感知+定位；`--hil` 强制 1x |

优先级：`--hil` > `--sensors-only` > `--profile`。HIL 遇闩锁非零退出。未隔离实机轮子禁止灌回放 cmd，除非 `HIL_ARMED=1`。

## 时钟与四元组

播放器按 `DualClock` 单调轴差值 `SleepNs`。墙钟只用于日志对齐。`git_sha` / `seed` 不一致 → `informal`，不得作为 PublishGate 证据。`env_version` 由 `scenario_loader` 绑到场景资产。

## 组件

| 模块 | 职责 |
|------|------|
| `autoloop.recorder replay` / `player` | 机上 / 边侧按 profile 重发 RawMessage |
| `autoloop.recorder verdict` | `FactualReplay` / `InterventionalReplay`；写 overlay |
| `cloud/simulation/replay` | 云端按 MCAP 推进时钟、投喂通道 |
| `cloud/simulation/scenarios` | `env_version` → 场景 |
| `regression_runner` | 固定 seed 跑 `regression_set` |
| `replay_tools` | seek、裁剪、两份 MCAP 对比 |

采集 flush 的 overlay 为 `pending_replay`（`valid=false`）。正式 L6：`verdict -a rec.mcap -b replay.mcap`。`ActionsFromMcap` 扫 topic 名含 `cmd_vel`；`ParseTwist` 先 `TwistStamped` 再 `Twist`。

## 与训练边界

- 模仿学习用转换后的 LeRobot，不在回放里改权重。
- 回归失败 → 该 `git_sha` 不得晋升；episode 进 `failure_pool`。
- re-roll 假设只写 sidecar，不覆盖原始 MCAP。
