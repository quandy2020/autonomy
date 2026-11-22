# Replay（可复现）

复现目标：同一 `VersionTuple` 下，把 MCAP 再喂进仿真或实机夹具，得到可对比的轨迹 / 任务结果。回放不是在线控制。

## 剖面

| Profile | 输入 | 用途 |
|---------|------|------|
| `open_loop` | 记录的 `/cmd_vel` 或 action | 核对执行器跟踪，不跑规划 |
| `mid_stack` | 记录的感知 / 定位输出 → 规划+控制 | SIL：代码 / 参数回归 |
| `sensor_in` | 记录的原始 lidar/camera/imu | 感知+定位回归；最重 |

默认 SIL 用 `mid_stack`。HIL 仅在台架，且 **禁止** 把回放 cmd 接到未隔离的实机轮子，除非显式 `HIL_ARMED=1`。

## 时钟

播放器按 `DualClock.mono_ns` 差值 `SleepNs`。墙钟只用于日志对齐。四元组不一致（尤其 `git_sha` / `seed`）的回放标记 `informal`，不得作为 PublishGate 证据。

## 组件

| 模块 | 职责 |
|------|------|
| `cloud/simulation/replay` | 按 MCAP 推进时钟、投喂通道 |
| `cloud/simulation/scenarios` | 把 `env_version` 绑到场景资产 |
| `regression_runner` | 固定 seed 跑 `regression_set` |
| `replay_tools` | CLI：seek、裁剪、对比两份 MCAP |

`autonomy.recorder` 的 `player` / `verdict` 可对同一 MCAP 做 L6 裁决；Autoloop 云端跑批量回归，不重复实现控制。

## 与训练的边界

- 模仿学习用转换后的 LeRobot，不在回放里改权重。
- 回归失败 → 该 `git_sha` 不得晋升；episode 进 `failure_pool`。
- re-roll 假设（「若当时不接管」）只写 sidecar 假设字段，不覆盖原始 MCAP。
