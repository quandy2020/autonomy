# Autoloop 数据闭环

端云飞轮：机上旁路采集 → 云端入库 / 转换 / 治理 / 训练 → 模型回灌机上。不改写 `autonomy` 规划 / 控制 / 感知，不插入 `/cmd_vel`。

机上证据 overlay 由 [`autoloop.recorder`](../autoloop/robot/recorder/README.md)（L0–L6）实现；`data_agent` 负责上传队列与遥测。

## 阅读顺序

| 编号 | 文档 | 内容 |
|------|------|------|
| 01 | [需求](01_requirements.md) | 目标、非目标、约束、能力面、触发优先级 |
| 02 | [方案](02_design.md) | 采集职责、身份、四元组、双时钟、P0–P3、治理、门禁 |
| 03 | [架构](03_architecture.md) | 端云拓扑、进程、分层、相邻域、故障隔离 |
| 04 | [数据流](04_dataflow.md) | 采集 → 上传 → 入库 → 训练 → 回灌；SIL / 可视化 |
| 05 | [使用](05_usage.md) | 构建、配置、launch、CLI、产物、最小检查 |
| 06 | [证据链](06_evidence_chain.md) | 主键、四元组、双时钟适配、哈希、日志反查 |
| 07 | [追查](07_investigation.md) | 根因卡片、家居失败展现、diagnostics 三类 |
| 08 | [回放](08_replay.md) | SIL / HIL profile、verdict、回归边界 |
| 09 | [可视化](09_visualization.md) | Evidence / 仪表 / 治理卡片 |
| 10 | [发布回灌](10_publishing.md) | PublishGate、原子 `current`、混版 |
| 11 | [安全域](11_safety.md) | 只闩锁、哈希链、与 monitor 共用文件 |
| 12 | [触发规则](12_triggers.md) | data_agent YAML + recorder TriggerEngine |
| 13 | [Schema 命名](13_schemas.md) | proto / JSONL / 转换 meta / include |
| 14 | [诊断](14_diagnostics.md) | 分流、告警、trace_index |
| 15 | [TODO](15_todo.md) | 成熟度、已决、实现缺口与验收 |
| 16 | [recorder 01 需求](recorder/01_requirements.md) | 机上 L0–L6 功能面 |
| 17 | [recorder 02 方案](recorder/02_design.md) | episode、触发、pipeline、L4/L6 |
| 18 | [recorder 03 架构](recorder/03_architecture.md) | 旁路进程拓扑 |
| 19 | [recorder 04 数据流](recorder/04_dataflow.md) | 采集 → sidecar → 回放 |
| 20 | [recorder 05 使用](recorder/05_usage.md) | `autoloop.recorder` CLI |

recorder 专文索引：[recorder/README.md](recorder/README.md)。

## 源码树对照

```text
autoloop/
  common/proto/          # VersionTuple DualClock EpisodeSidecar TraceMetadata
  common/base/           # path/time/hash/json → autolink
  robot/recorder/        # L0–L6 证据进程 autoloop.recorder
  robot/data_agent/      # ring / trigger / snapshot / uploader / latch
  robot/model_sync/      # 拉模型、原子切 current
  robot/config/          # data_agent YAML + recorder.pb.txt
  robot/bringup/launch/  # recorder / data_agent / system
  cloud/ingestion/       # record_receiver 哈希+四元组准入
  cloud/conversion/      # MCAP→LeRobot  autoloop-convert
  cloud/governance/      # success / failure / diagnostics / 人审
  cloud/datasets/        # lerobot_builder / regression_set
  cloud/training/        # Python 训练入口（不在采集热路径）
  cloud/publishing/      # model_registry / model_publisher
  cloud/diagnostics/     # trace_index / replay_tools / regression_runner
  cloud/observability/   # dashboards / alerts / telemetry_gate
  cloud/simulation/      # replay / scenarios
```

## 不变量

1. **旁路**：`RawMessage` + BEST_EFFORT + KEEP_LAST；采集不发布控制话题。
2. **准入**：`policy_version` / `env_version` / `git_sha` / `seed` 任一空 → `admitted=false`，进诊断，不进训练 / 正式回放 / 发布。
3. **安全**：L4 只闩锁。编译期 `AUTOLOOP_LATCH_ONLY=1`。制动由 `autonomy.monitor`。
4. **身份**：产物用 `robot_id` + `episode_id` + `trace_id` + `content_sha256` 串联。
5. **因果边界**：在线因果是时间窗 + 通道共现，不是物理力传递；物理复现走 SIL / HIL。
