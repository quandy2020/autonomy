# 02 方案

采集与证据分离：在线只做旁路缓冲、触发落盘与上传排队；L2–L6 与云端治理在 episode 关闭之后消费 sidecar。安全监控是独立路径，与采集共享闩锁文件和哈希格式，不共享控制通道写权限。

## 容器与载荷

| 角色 | 选择 | 理由 |
|------|------|------|
| 容器 | MCAP | 可崩溃恢复；autoviz Record 已能播。 |
| 载荷 | Protobuf 字节 | 复用 automsgs；采集路径不二次编码。 |
| 证据头 | `EpisodeSidecar` protobuf + JSON 镜像 | 机器链路 `.pb`；清单 / UI `.json`。 |
| 事件 | JSONL 一行一事 | 禁止二进制 payload；通道字节只在 MCAP。 |

## 采集职责（已决）

| 进程 | 职责 | 不职责 |
|------|------|--------|
| `autoloop.recorder` | L1–L6 证据引擎：旁路订阅、滚动窗、触发、MCAP、sidecar、pipeline、闩锁、回放裁决 | 不调云 API、不切模型 |
| `data_agent` | 上传队列、断点续传、遥测心跳、结构化日志、crash 落盘 | 不第二套事件图；长期只消费 recorder 产物 |
| `model_sync` | 拉制品、校验 sha256、任务间隙切 `current` | 不在 RUNNING 热切 |

过渡期允许 `data_agent` 自写 `snapshot.mcap`（`autoloop.common.EpisodeSidecar`）。云端 `record_receiver` **必须同时接受**：

| 生产者 | 目录 / stem | MCAP 名 | sidecar |
|--------|-------------|---------|---------|
| recorder | `$output_dir/episodes/<id>.*` | `<id>.mcap` | `<id>.sidecar.pb`（evidence） |
| data_agent | `$record_root/ep-<utc_ns>-<kind>/` | `snapshot.mcap` | `sidecar.pb`（common） |

入库后统一成归档布局 `archive/{robot_id}/{episode_id}/snapshot.mcap` + `sidecar.pb`（common）。recorder DualClock 在适配器里映射为 `mono_ns`/`utc_ns`。

## 主键

```text
robot_id + episode_id
  ├─ VersionTuple { policy_version, env_version, git_sha, seed }
  ├─ DualClock    段起止
  ├─ content_sha256                      mcap 字节
  ├─ trace_id / span_id                  日志与事件
  └─ admitted / missing_fields
```

`episode_id` 约定 `ep-<utc_ns>-<kind>`。归档键：`robot_id / episode_id / content_sha256`。`policy_version` 禁止只写 `latest`，建议 `<name>-<git_sha前8>-<date>`。

`trace_id` 在打开 episode 时生成，写入 glog 前缀、`events.jsonl`、sidecar。云端 `trace_index` 倒排：`trace_id` → `episode_id` → 归档路径。

## 版本四元组

`autoloop.common.VersionTuple` 与 recorder 字段同名。解析顺序：`AUTOLOOP_*` → `AUTONOMY_*` → 编译期 `AUTOLOOP_GIT_SHA`。

| 环境变量 | 字段 |
|----------|------|
| `AUTOLOOP_POLICY_VERSION` / `AUTONOMY_POLICY_VERSION` | `policy_version`（优先） |
| `AUTONOMY_SOFTWARE_VERSION` | `policy_version` 回退 |
| `AUTOLOOP_ENV_VERSION` / `AUTONOMY_ENV_VERSION` | `env_version`（地图 + 标定） |
| `AUTOLOOP_SEED` / `AUTONOMY_SEED` | `seed` |
| `AUTOLOOP_GIT_SHA` / `AUTONOMY_GIT_SHA` | `git_sha` |

`IsAdmitted` 为假：`admitted=false`，`missing_fields` 列出空字段，落盘进 diagnostics；`record_receiver` 拒收进训练池。可视化仍可读。

## 双时钟

| 方言 | 单调轴 | 墙钟 | 使用处 |
|------|--------|------|--------|
| `autoloop.common.DualClock` | `mono_ns` | `utc_ns` | data_agent sidecar、云端 ingest |
| recorder `evidence.proto` | `monotonic_ns` | `wall_utc_ns` | `autoloop.recorder` sidecar |

单调轴：环内排序、触发超时、ring 剪枝、LeRobot 对齐、回放 `SleepNs`。墙钟：跨机清单、仪表、人审。实现优先 `autolink::Time::MonoTime` / `Now`。回放不得用墙钟当仿真时间。**已决**：云端与 data_agent 以 `autoloop.common.DualClock` 为准。recorder 落盘保持 `monotonic_ns`/`wall_utc_ns`（含 `time_source`）；`record_receiver` 适配器做字段拷贝，禁止用两套名字做 join。

## 在线触发

`data_agent` 规则面（`robot/config/data_agent/*.yaml` 窗口）：

| `TriggerKind` | 信号 | 默认优先级 |
|---------------|------|------------|
| `kEstop` | 闩锁文件 | P0 |
| `kTaskFail` | `kSampleFeedback` | P0 |
| `kTakeover` | `kSampleTeleop` | P0 |
| `kLocDegrade` | 定位质量边沿 | P1 |
| `kManual` | `trigger_snapshot.sh` | P2 |
| `kPeriodic` | 心跳 | P3 |

recorder 另含 BT recover、路径偏离、通道超时、`--full-dump` 会话；详见 [recorder 02](recorder/02_design.md)。命中后 **拷贝** ring 窗口落盘，热路径不 `TakeAll`，以免丢后续秒级上下文。窗口三套：`conservative` 2s / `aggressive` 4s / `minimal` 1s。

## P0–P3 上传

| 优先级 | `upload` | `policy` |
|--------|----------|----------|
| P0 | 是，可抢占 P3 | `immediate` |
| P1 | 是 | `upload_not_expert` |
| P2 | 抽样或人工快照 | `p2_sample_keep` / `drop` |
| P3 | 分层抽样 | `p3_stratified_keep` / `drop` |

`uploader` 只把本地目录拷到 staging。密钥与云 API 不进采集进程。未 admitted 仍可上传供人看，训练拒绝。

## 治理分流

```text
auto_label → human_review（抽检 / 争议）
    ├─ success_pool   可训练正例
    ├─ failure_pool   根因 / 回归 / 不进模仿
    └─ diagnostics    未准入、破哈希、缺观测、混版
```

人审队列键：`episode_id`。无 `evidence_node_id` 的观察不进 L2 归因。P1 接管默认不进 `teleop_imitation` 专家集，除非人审放行。

## 转换

MCAP → LeRobot，且 **meta 必须带回 `episode_id` 与 VersionTuple**，否则不得进 `lerobot_builder`。

| 步骤 | 模块 |
|------|------|
| 通道映射 | `field_mapping`（odom / cmd / image / state） |
| 时钟 | `clock_alignment` 用单调轴 |
| 视频 | `video_encoder`（可选，meta 仍指原 episode） |
| 表 | `parquet_writer` + `lerobot_layout` |

preset：`teleop_imitation` / `navigation_imitation` / `perception_only`（`cloud/conversion/presets/`）。recorder 边侧 `convert` 写 JSONL 而不写 parquet，作为轻量导出；云端 `autoloop-convert` 走完整布局。

## 事件与日志

事件 JSONL / proto 一行一事：

| 字段 | 用途 |
|------|------|
| `ts_mono_ns` / `ts_utc_ns` | 双时钟；回放只用 mono |
| `trace_id` / `span_id` | 回溯到 episode |
| `event_code` | heartbeat / snapshot / upload / latch / drop / crash / model_rollback |
| `severity` | 仪表密度 |
| `robot_id` / `episode_id` | 主键 |
| `payload` | JSON object |

glog 前缀：`[ts_utc][ts_mono][trace_id][file:line]`。热路径禁止 `LOG(INFO)` 打印通道 payload。crash dump → `robot/log/`，`log_rotate.sh` 不删未上传 P0。

从一条 ERROR 反查：前缀 `trace_id` → `trace_index` → sidecar 四元组 → MCAP 在 `[start, end]` 内 seek。

## L4 闩锁

`safety_guard` / recorder `SafetyMonitor`：已闩 → Pass；teleop → Handover（只记账）；超 `max_cmd_vx` / `max_cmd_wz` → Fallback，只 `SetLatched`，不向 `/cmd_vel` 写零速。闩保持到人工 `Clear()`。安全事件追加 SHA-256 链（`common/base/hash_util`）；云端 Verify 失败 → `safety_event` 告警，该 robot 制品不得晋升。MRM 仍由 `autonomy.monitor` 读闩停机。

闩锁文件：`AUTOLOOP_SAFETY_LATCH_PATH` → `AUTONOMY_SAFETY_LATCH_PATH` → `~/.autonomy/runtime/safety_latch`（与 monitor / Bridge 共用）。

## L6 回放

| Profile | 输入 | 用途 |
|---------|------|------|
| `open_loop` | 记录的 `/cmd_vel` | 核对执行器跟踪；仅离线、无活 controller |
| `mid_stack` | 感知 / 定位输出 → 规划+控制 | 默认 SIL：代码 / 参数回归 |
| `sensor_in` | 原始 lidar / camera / imu | 感知+定位回归；HIL 强制 1x，闩锁则拒绝 |

四元组不一致（尤其 `git_sha` / `seed`）标记 `informal`，不得作为 PublishGate 证据。re-roll 假设只写 sidecar，不覆盖原始 MCAP。HIL 禁止把回放 cmd 接到未隔离的实机轮子，除非显式 `HIL_ARMED=1`。

## 发布门禁与回灌

同时满足才允许 `model_publisher` 更新 registry `current`：

1. 构建 `git_sha` 与制品一致。
2. `regression_set` 全绿（固定 seed）。
3. 安全链近期无未解释断裂。
4. 转换 / 训练元数据含完整四元组模板（新 `policy_version`）。
5. 人审：failure_pool 阻断单已关闭。
6. recorder `PublishGate`：回放 valid、归因覆盖、回归 δ、安全链（机上 `gate/latest.json` 为其边侧镜像）。

未过门禁进 `candidates/`，机上默认不拉。

机上 `model_sync`：拉清单 → 核 sha256 → 写入 `cache/<version>` → `LocalRegistry` 切指针。

**已决（原子切换）**：写 `next` → `fsync` 目录 → `rename(next, current)`；失败不覆盖旧 `current`。当前实现若直接写 `current`，视为 TODO，验收以 rename 为准。**RUNNING 任务中禁止切换**。回滚同样原子，并打 `event_code=model_rollback`。

新 episode 必须：`policy_version == LocalRegistry.Current()`，`git_sha` 为运行中二进制，`env_version` 为当前地图 / 标定。字段非空但与 `current` 不等 → telemetry **混版**，不得作为「新模型」成功样本。

## 命名

- Include：`#include "common/..."`, `"robot/..."`, `"cloud/..."`（`AUTOLOOP_SRC_ROOT`）。
- 目标名：`autoloop_<domain>_<unit>`；测试 `<unit>_test.cpp` + `autoloop_test()`。
- Proto package：`autoloop.common`；recorder proto：`autoloop.robot.recorder.proto`。
- 配置 YAML 蛇形；C++ 类型 PascalCase。
