# 02 方案

采集与证据分离：在线只做旁路缓冲与触发落盘；L2–L6 在 episode 关闭时（或 `pipeline` 子命令）对 sidecar 运行。安全监控是第二条独立进程，与 recorder 共享闩锁文件和哈希链格式，不共享控制通道写权限。

## 容器与载荷

| 角色 | 选择 | 理由 |
|------|------|------|
| 容器 | MCAP（最小 unchunked writer） | 可崩溃恢复（`RecoverMcap`）；autoviz Record 面板已能播 MCAP。 |
| 载荷 | Protobuf 字节 | 复用 automsgs schema；采集路径写 `RawMessage::message`，不二次编码。 |
| 证据头 | `EpisodeSidecar` protobuf + JSON 镜像 | 机器可读 `.pb`；autoviz / 清单用 `.json`。 |

## 版本四元组

`VersionTuple`：`policy_version`、`env_version`、`git_sha`、`seed`。解析顺序：环境变量覆盖 conf，git SHA 还可从 `GetGitCommitID()` 文本（`Autonomy git commit ID: <sha> on <date>`）提取。

| 环境变量 | 字段 |
|----------|------|
| `AUTONOMY_POLICY_VERSION` | `policy_version`（优先） |
| `AUTONOMY_SOFTWARE_VERSION` | `policy_version` 回退（与 fullstack record 脚本同名） |
| `AUTONOMY_ENV_VERSION` | `env_version` |
| `AUTONOMY_SEED` | `seed` |
| `AUTONOMY_GIT_SHA` | `git_sha` |

`IsAdmitted` 为假时 `EpisodeHeader.admitted=false`，落盘目录为 `$output_dir/diagnostics/`。`require_version_tuple` 为真时同样拒绝训练队列。

## 双时钟

`DualClock`：`monotonic_ns`（`CLOCK_MONOTONIC`）+ `wall_utc_ns`（`CLOCK_REALTIME`）+ `time_source`。默认 software。若 `RawMessage.timestamp != 0`，旁路订阅用其覆盖 monotonic。episode 内排序与触发超时以 monotonic 为准；墙钟用于跨机对齐与清单。

## Episode 边界

打开：BT 任务根（Navigate / Localization / Mapping / Charging / Tracking / Exploration / Voice）IDLE→RUNNING，或任意 `/autonomy/task/*/goal`、`/goal_pose`。关闭：该根 SUCCESS/FAILURE、recover、任务 feedback 失败、进程 `Shutdown`。`--full-dump` 为会话模式：Start 即打开，Shutdown 才关闭，期间按 `flush_period_sec` 增量写 MCAP。

滚动窗口默认 `buffer_sec=30`。默认路径触发才快照；全量路径靠周期 flush 在剪枝前把窗口写入磁盘。

## 在线触发

`TriggerEngine` 在订阅回调与 `SpinOnce` 超时轮询中产生 `TriggerEvent`：

- `OnSafetyLatch`：急停文件闩。
- `OnBtStatus`：BT recover；Navigate / Localization / Mapping / Charging / Tracking / Exploration / Voice 的 FAILURE。
- `OnTaskFailure`：`/autonomy/task/*/feedback` 为 FAILED / CANCELED（tracking 含 TARGET_LOST）。
- `OnLocalizationQuality`：`localization_quality` 低于 `localization_quality_floor`（边沿触发）。
- `OnPathDeviation`：odom 相对 `/plan` 超过 `path_deviate_m`。
- `OnCost`：仅当 `cost_channel` 非空。
- `OnTakeover`：`teleop_goal_channel` 出现消息。
- `PollTimeouts`：通道 `timeout_sec > 0` 且超过最后见到的 monotonic。

`/cmd_vel` 仅用于把 Twist 记为 `ActionEvent`（L6 匹配），不改写控制。

## 离线主问题（单 episode）

`RunEvidencePipeline` 对 `EpisodeBundle` 一次做完：

1. `BuildEventGraph(transitions, triggers)`
2. 过滤 `AcceptObservation`（须有 description 与至少一个 `evidence_node_id`）
3. `RankRootCauses`；若提供 `reroll` 则 `RerollHypothesis`
4. `AttributeBtFailure`
5. 将安全事件封入哈希链并 `Verify`
6. `replayed_actions` 为空 → `PendingReplay`；否则事实/干预回放
7. `EvaluateGate`：在 `history`（`$output_dir/metrics/episodes.jsonl` 最近 256 条）+ 当前 episode 上算 MPS/MPD/覆盖；回归、re-roll、归因覆盖、安全链、回放五闸

在线 flush 不把 `recorded_actions` 复制成回放结果。`replayed_actions` 为空时 sidecar 写 `reason=pending_replay`、`valid=false`，发布门禁在回放闸失败。正式 L6 用 `verdict -a rec.mcap -b replay.mcap`。flush 另写 `gate/latest.json`；`autonomy.recorder gate -d <records>` 以 `publish_allowed` 为退出码。

P0 episode 若设置 `AUTONOMY_LOG_ROOT`，在 `$AUTONOMY_LOG_ROOT/events/recorder_<id>/manifest.json` 写危险指针（指向 mcap / sidecar），不改 EventBundler API。

## P0–P3 上传策略

`DecideUpload(priority, episode_id, scene, p2, p3)`：

| 优先级 | `upload` | `policy` |
|--------|----------|----------|
| P0 | 是 | `immediate` |
| P1 | 是 | `upload_not_expert` |
| P2 | 哈希抽样 `< p2_sample_rate` | `p2_sample_keep` / `drop` |
| P3 | 哈希抽样 `< p3_sample_rate` | `p3_stratified_keep` / `drop` |

`upload` 子命令只拷贝 `admitted && upload` 的 stem 文件集，不传输密钥、不调用云 API。

## L4 闩锁方案

`EvaluateCommand`：已闩 → Pass；teleop → Handover（只记账）；`|vx|>max_cmd_vx` 或 `|wz|>max_cmd_wz` → Fallback。Fallback 调用 `SafetyLatch::SetLatched(true, "safety_monitor_cmd_limit")`，不向 `/cmd_vel` 写零速。哈希链追加 `cmd_limit` / `handover`，持久化 `safety/chain.bin`。

## L6 回放

| 模式 | 入口 | 语义 |
|------|------|------|
| `open_loop` | `replay --profile open_loop` | 按 log 时间重发全部 topic（含 `/cmd_vel`）；仅离线、无活 controller。 |
| `mid_stack` | `--profile mid_stack` 或 `--sensors-only` | 对齐 `replay_play.sh -k`：不重发 `cmd_vel*`。 |
| `sensor_in` | `--profile sensor_in` 或 `--hil` | 再跳过 `/plan`、BT、task、`/perception/*`。`--hil` 强制 1x，闩锁则拒绝。 |
| 事实裁决 | `verdict` 默认 | `FactualReplay`；长度一致视为 `bit_exact`。 |
| 干预裁决 | `verdict --interventional` | `InterventionalReplay`。 |

## 观察写回

`AppendObservationJsonl` / autoviz Observe 页：无 description 或无 `evidence_node_ids` 则拒绝。flush 时加载 `$episode.observations.jsonl` 并入 sidecar。
