# Evidence chain（可回溯）

回溯目标：给定任意失败现场（日志一行、仪表一个点、一段视频），能在一次跳转内落到 **同一 episode 的 MCAP + sidecar + 四元组**。

## 主键

```
robot_id + episode_id
    ├─ VersionTuple { policy_version, env_version, git_sha, seed }
    ├─ DualClock    { mono_ns, utc_ns }   段起止
    ├─ content_sha256                      mcap 字节
    ├─ trace_id / span_id                  日志与事件
    └─ admitted / missing_fields
```

`trace_id` 在 `data_agent` 打开 episode 时生成，写入 glog 前缀、`events.jsonl`、sidecar。云端 `trace_index` 建倒排：`trace_id` → `episode_id` → 归档路径。

## 四元组

与 `autonomy.recorder` 同字段。解析顺序：`AUTOLOOP_*` → `AUTONOMY_*` → 编译期 `AUTOLOOP_GIT_SHA`。

| 字段 | 含义 | 缺则 |
|------|------|------|
| `policy_version` | 当前 `model_sync` 指针 / 软件版本 | 无法证明「跑的是哪套策略」 |
| `env_version` | 地图 + 标定版本 | 无法对齐仿真场景 |
| `git_sha` | 机上二进制 | 无法检出同代码做 SIL |
| `seed` | 随机性 | 无法回归 |

`IsAdmitted` 为假：落盘到 diagnostics，云端 `record_receiver` 拒收进训练池。可视化仍可读。

## 双时钟

- **单调轴** `mono_ns`：环内排序、触发超时、ring 剪枝、LeRobot 对齐。
- **UTC** `utc_ns`：跨机清单、仪表、人审。
- 实现优先 `autolink::Time::MonoTime` / `Now`。

回放不得用墙钟当仿真时间。

## 哈希链

| 对象 | 算法 | 谁写 / 谁验 |
|------|------|-------------|
| `snapshot.mcap` | SHA-256 hex → `content_sha256` | snapshot_writer / hash_verifier |
| 安全事件 | 追加链 `prev_hash` → `entry_hash` | safety_guard / 云端 Verify |
| 上传分片 | 文件哈希 + offset 书签 | resumable_upload |

Autolink 无哈希 API，使用 `common/base/hash_util`。

## 日志可回溯

glog 前缀 `[utc][mono][trace_id][file:line]`。禁止热路径把通道 payload 打进 INFO。crash 由 `crash_handler` 落 `log/`，rotate 不删未上传 P0。

从一条 ERROR 反查：

1. 前缀 `trace_id` → `trace_index`
2. sidecar 四元组 → 检出同 `git_sha` 做 SIL
3. MCAP 在 `start.mono_ns … end.mono_ns` 内 seek
