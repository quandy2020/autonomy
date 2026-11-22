# 06 证据链（可回溯）

回溯目标：给定任意失败现场（日志一行、仪表一个点、一段视频），一次跳转到 **同一 episode 的 MCAP + sidecar + 四元组**。方案决议见 [02](02_design.md)；本篇展开主键、哈希与日志反查。

## 主键

```text
robot_id + episode_id
  ├─ VersionTuple { policy_version, env_version, git_sha, seed }
  ├─ DualClock    start / end（common: mono_ns, utc_ns）
  ├─ content_sha256
  ├─ trace_id / span_id
  └─ admitted / missing_fields / priority
```

归档键：`robot_id / episode_id / content_sha256`。训练样本必须能从 LeRobot `meta` 反查该键。`episode_id` = `ep-<utc_ns>-<kind>`；`kind` 用 `TriggerKind` 整型或 recorder 内部 id，入库时规范化为字符串。

`trace_id` 在打开 episode 时生成（UUID 或 `robot_id:utc_ns`），写入：glog 前缀、`events.jsonl`、sidecar、P0 manifest。云端 `trace_index` 倒排 `trace_id | episode_id | git_sha → archive_path`。

## 四元组语义

| 字段 | 证明什么 | 缺则 |
|------|----------|------|
| `policy_version` | 跑的是哪套策略 / `model_sync.current` | 无法归因模型 |
| `env_version` | 地图 + 标定 | 无法对齐仿真场景 |
| `git_sha` | 机上二进制 | 无法检出同代码做 SIL |
| `seed` | 随机性 | 无法回归 |

解析：`AUTOLOOP_*` → `AUTONOMY_*` → 编译期 `AUTOLOOP_GIT_SHA`。`policy_version` 可回退 `AUTONOMY_SOFTWARE_VERSION`。禁止 `latest`。

`IsAdmitted`：四字段皆非空。为假则 `admitted=false`，`missing_fields` 列出空名，落盘 diagnostics；可视化可读，训练 / 正式回放 / 发布拒绝。

## 双时钟与适配

| 层 | 单调 | 墙钟 | 其它 |
|----|------|------|------|
| recorder 落盘 | `monotonic_ns` | `wall_utc_ns` | `time_source` |
| common / 云 | `mono_ns` | `utc_ns` | 无 |
| 实现 | `autolink::Time::MonoTime` | `Now` | software 默认 |

适配器（入库必做）：

```text
common.mono_ns     = rec.monotonic_ns
common.utc_ns      = rec.wall_utc_ns
丢弃 time_source 或写入 sidecar JSON 扩展字段 time_source
```

回放只用单调轴差值 `SleepNs`。墙钟只用于清单与仪表。

## 哈希

| 对象 | 算法 | 谁写 | 谁验 |
|------|------|------|------|
| MCAP 文件 | SHA-256 hex → `content_sha256` | snapshot_writer / recorder flush | `hash_verifier` |
| 安全事件 | `prev_hash → entry_hash` | safety_monitor / safety_guard | `verify-chain` / 云端 Verify |
| 上传分片 | 文件哈希 + offset 书签 | `ResumeCopy` | 续传前比对 |

Autolink 无哈希 API，用 `common/base/hash_util`。哈希失败 → 拒收 + 告警 `data_loss`，可 `RecoverMcap` 后重传，不得静默改 `content_sha256`。

## 日志反查

glog 前缀：`[ts_utc][ts_mono][trace_id][file:line]`。热路径禁止把通道 payload 打进 INFO。crash → `robot/log/`，`log_rotate.sh` 不删未上传 P0。

步骤：

1. ERROR 行取 `trace_id`
2. `trace_index` → `episode_id` + 归档路径
3. sidecar 四元组 → 检出同 `git_sha` 做 SIL
4. MCAP 在 `[start.mono_ns, end.mono_ns]` seek
