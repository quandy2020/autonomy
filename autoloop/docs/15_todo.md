# 15 TODO 与实现成熟度

方案以 01–14 与 [recorder/](recorder/README.md) 为准。本篇列出相对方案尚未落地的工作；**不改变** L0 规划 / 控制 / 感知。验收写「完成」的判据，禁止只把 `Init()` 置 true 当作完成。

## 成熟度

| 域 | 状态 | 说明 |
|----|------|------|
| `autoloop.recorder` L1–L6 | 源码完成 | 采集 / sidecar / pipeline / 闩锁 / CLI；未在本仓库强制 build |
| `common` proto + hash/path/time | 可用 | DualClock 与 recorder 字段名未统一（由 ingest 适配） |
| `data_agent` ring / trigger / snapshot / uploader | 骨架可用 | 第二套 MCAP 方言；长期应收口到 recorder 产物 |
| `model_sync` | 部分 | `SwitchTo` 直接写 `current`，缺 next+rename 与 RUNNING 闸 |
| `cloud/*` 多数单元 | `Init()` 桩 | 接口目录在，流水线未接 |
| `cloud/training/*.py` | CLI 占位 | 打印函数名，未调 LeRobot |
| loop-runtime drivers/loc/nav/perc | `Init()` 桩 | 不替代 `autonomy/` 栈 |

## 已决（不要再开设计争议）

1. recorder 是 L1–L6 证据引擎；data_agent 做上传 / 遥测；ingest 同时收两套落盘并归一。
2. 云端 DualClock 用 `mono_ns`/`utc_ns`；recorder 落盘名保持，适配器映射。
3. `LocalRegistry` 契约：`next` → fsync → rename `current`。
4. L4 只闩锁；闩文件与 monitor 共用 `~/.autonomy/runtime/safety_latch`。
5. 训练 / 正式回放 / 发布只消费 `admitted=true`。

## 机上 TODO

| ID | 项 | 现状 | 验收 |
|----|-----|------|------|
| T-CAP-01 | ingest 适配 recorder `<id>.mcap` + evidence sidecar → common sidecar | `record_receiver` 只读 `snapshot.mcap`/`sidecar.pb` | 两种目录都能 `Ingest` 成功并写出 `archive/{robot_id}/{episode_id}/` |
| T-CAP-02 | data_agent 默认上传 recorder `episodes/` + manifest | 自写 snapshot | `PumpOnce` 能入队 recorder stem 六件套 |
| T-CLK-01 | DualClock 适配器 | 两套字段名 | 单测：`monotonic_ns`→`mono_ns`，`wall_utc_ns`→`utc_ns` |
| T-SYNC-01 | `LocalRegistry` 原子切 | 直接写 `current` | crash 在 rename 前旧指针仍可读 |
| T-SYNC-02 | RUNNING 禁止切换 | 无任务状态闸 | RUNNING 返回 error，IDLE 成功 |
| T-UP-01 | 断点续传书签持久化 | `ResumeCopy` 内存 offset | 进程重启从书签续；P0 抢占 P3 |
| T-UP-02 | 密钥隔离 | 采集进程无云 API | 上传只拷 staging |
| T-LOG-01 | glog 前缀 `trace_id` | 未统一 | ERROR 行可被 `log_ingestion` 解析 |
| T-LOG-02 | crash_handler + rotate 保 P0 | 脚本占位 | 未上传 P0 不被 `log_rotate.sh` 删 |
| T-TEL-01 | 心跳发布 `DataAgentHealth` | 结构体有，未出总线 | 仪表能读 ring/upload_pending/latch |
| T-SAFE-01 | `safety_guard` 写同一闩文件 | 内存 `latched_` | 与 recorder `SafetyLatch` 路径一致 |

## 云端入库 / 转换 TODO

| ID | 项 | 现状 | 验收 |
|----|-----|------|------|
| T-IN-01 | `hash_verifier` 对两套 MCAP 名 | 写死 `snapshot.mcap` | recorder stem 也能验 |
| T-IN-02 | `dedup` 按 `content_sha256` + 场景 | `Init()` 桩 | 相同哈希不进第二份训练样本 |
| T-CV-01 | `Converter` 真正读 MCAP 写 LeRobot | `RunConvert` 只 `Init`+mkdir | `autoloop-convert convert` 产出带四元组的 meta |
| T-CV-02 | `field_mapping` 对齐 automsgs odom/cmd/image | 桩 | preset YAML 可跑通 navigation_imitation |
| T-CV-03 | `clock_alignment` 单调轴 | 桩 | 帧时间无墙钟回退 |
| T-CV-04 | `parquet_writer` | stub 注释 | 写出可读 parquet 或明确降级 JSONL 并在 check 声明 |
| T-CV-05 | `video_encoder` | stub | 可选；meta 仍含 `episode_id` |
| T-CV-06 | `RunCheck` 验 hash+四元组 | 部分 | 缺字段 / 破哈希非零退出 |

## 治理 / 数据集 / 训练 TODO

| ID | 项 | 现状 | 验收 |
|----|-----|------|------|
| T-GV-01 | `AutoLabeler` 按 TriggerKind / Priority | 桩 | P0 失败 → failure_pool；P3 成功 → success 候选 |
| T-GV-02 | `ReviewQueue` 持久化 `episode_id` | 桩 | 重启不丢争议单 |
| T-GV-03 | success/failure/diagnostics 三池 | 桩 | 互斥；diagnostics 不可被误标进 success |
| T-GV-04 | `MetadataStore` 检索 sidecar 字段 | 桩 | 按 `git_sha` / `policy_version` 列出 episode |
| T-DS-01 | `DatasetBuilder` 拒无四元组 meta | 桩 | 缺字段 `Init` 后 Build 失败 |
| T-DS-02 | `RegressionSet` 从 failure_pool 取样 | 桩 | 固定 seed 可复现集合成员 |
| T-TR-01 | `train_lerobot.py` 调真实训练入口 | `print` | 读 preset YAML，产物写入 registry 布局 |
| T-TR-02 | `train_perception.py` / `eval_policy.py` | 占位 | 同样带四元组模板 |

## 发布 / 回放 / 诊断 / 观测 TODO

| ID | 项 | 现状 | 验收 |
|----|-----|------|------|
| T-PB-01 | `ModelRegistry.Put` + `candidates/` + `current` | 桩 | 未过门禁不可被 sync 拉到 current |
| T-PB-02 | `ModelPublisher` 接 PublishGate 六闸 | 桩 | 缺回归或安全链断裂不打 current |
| T-SIM-01 | `ReplayController` 按 mono_ns SleepNs | 桩 | 与 recorder replay 同一 profile 语义 |
| T-SIM-02 | `ScenarioLoader` 绑 `env_version` | 桩 | 未知 env 拒绝正式回归 |
| T-DG-01 | `TraceIndex` 倒排 | 桩 | 一条 glog `trace_id` 命中归档路径 |
| T-DG-02 | `ReplayTool` seek/裁剪/对比 | 桩 | 两份 MCAP cmd 序列可 diff |
| T-DG-03 | `RegressionRunner` 跑 regression_set | 桩 | 失败阻止 T-PB-02 |
| T-DG-04 | `MonitorBridge` health+latch | 桩 | 仪表可见 latch_set |
| T-OB-01 | `LogIngestion` 解析前缀 | 桩 | 写入 trace_index |
| T-OB-02 | `TelemetryGate` 混版标记 | 桩 | 混版 episode 不可当新模型正例 |
| T-OB-03 | dashboards JSON 填 query | 仅 `title` | Grafana 可导入并下钻 episode |
| T-OB-04 | alerts YAML 接通知 | 仅文件名级 | payload 含 `trace_id` |

## 文档 / 工程 TODO

| ID | 项 | 现状 | 验收 |
|----|-----|------|------|
| T-DOC-01 | DualClock 字段最终是否改 proto 合并 | 适配器方案已决 | 若合并，同步改 common 与 recorder 并删适配器 |
| T-DOC-02 | 嵌套 `autoloop` git 仓库 | 现为 autonomy 子树；`.gitmodules` 预置 | 独立远程时 submodule 指向 civetweb/mcap |
| T-BLD-01 | 工作区联编 `autoloop.recorder` | 用户当前不要求 build | 打开 build 开关后 `autoloop.recorder` 与 `autoloop-convert` 链过 |

## 明确不做

| 项 | 理由 |
|----|------|
| 改 `autonomy` planning/control/perception | L0 不变量 |
| 采集进程发布 `/cmd_vel` | L4 单向 |
| 云端远程刹停 | monitor / MRM 职责 |
| 任务 RUNNING 热切模型 | 混版风险 |
| 可视化写控制通道 | 只读投影 |
| Autoloop 树内 drivers 替换实机驱动 | loop-runtime 接口，非产品栈 |
