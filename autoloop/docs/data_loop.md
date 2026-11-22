# Data loop

闭环是状态机，不是单向 ETL。每步写入可检索身份，下一步只消费 `admitted=true` 的对象。

## 主路径

```
1 旁路订阅  →  2 ring 窗口  →  3 触发  →  4 MCAP+sidecar
        →  5 P0–P3 上传  →  6 云端校验归档  →  7 解析/去重
        →  8 转换 LeRobot  →  9 治理分流  →  10 训练或回归
        →  11 PublishGate  →  12 registry  →  13 model_sync 原子切换
        →  14 下一 episode 的 policy_version = 新版本
```

### 1–4 机上捕获

- 订阅 Autolink `RawMessage`（BEST_EFFORT + KEEP_LAST），拷贝预序列化字节，不二次编码。
- `ring_buffer` 按单调时钟剪枝；默认窗口见 `config/data_agent/*.yaml`（conservative 2s）。
- `trigger_manager` 上升沿命中后 **拷贝** 窗口落盘（`Snapshot`），热路径不 `TakeAll`，以免丢后续秒级上下文。
- `snapshot_writer` 写 `snapshot.mcap` + `sidecar.pb`；`content_sha256` 对 mcap 文件。
- `--full-dump` / 测试全量：会话级打开，周期 flush，Shutdown 才关 episode。

### 5 上传

| 优先级 | 策略 |
|--------|------|
| P0 急停 / 任务失败 / 闩锁 | 立即、可抢占 P3 |
| P1 定位降级 / 接管 | 上传，不进专家演示集 |
| P2 人工快照 | 默认上传 |
| P3 周期 / 成功抽样 | 分层抽样 |

`uploader` 只拷贝本地目录到 staging；密钥与云 API 不进采集进程。四元组不齐无论 P 级都进诊断，仍可上传供人看，但 `admitted=false`。

### 6–7 入库

`record_receiver`：读 sidecar → `IsAdmitted` → `Sha256File(mcap)==content_sha256` → `archive_store` 不可变拷贝。失败写 `observability/alerts`。`record_parser` 按 Autoloop 最小 MCAP 方言抽通道；`dedup` 按内容哈希 + 场景聚类去重。

### 8 转换

`mcap_to_lerobot`：DualClock 单调轴对齐 → field_mapping → parquet / 视频。preset：`teleop_imitation` / `navigation_imitation` / `perception_only`。转换后 meta 必须保留 `episode_id` 与四元组，否则不得进 `lerobot_builder`。

### 9 治理

```
auto_label → human_review（抽检 / 争议）
    ├─ success_pool   可训练正例
    ├─ failure_pool   根因 / 回归 / 不进模仿
    └─ diagnostics    未准入、破哈希、缺观测
```

人审队列键：`episode_id`。无 `evidence_node_id` 的观察不进 L2 归因。

### 10–13 学习与回灌

训练入口在 `cloud/training/`（Python），不在采集热路径调度作业。产出写入 `model_registry`；`PublishGate` 通过后 `model_publisher` 打版本。机上 `model_sync` 下载到 cache，写 `current` 指针（先写 `next` 再 rename）。任务 RUNNING 期间禁止切换。

### 14 闭环闭合

新 episode 的 `policy_version` 必须等于机上 `current`。若 sync 失败，下一圈仍用旧版本，telemetry 报 `model_sync` 错误，不静默混版。

## 反路径（必须存在）

| 事件 | 路径 |
|------|------|
| 缺四元组 | diagnostics + 可视化仍可打开 MCAP |
| 训练指标回退 | 不发布；regression_set 加该 episode |
| 回灌后接管率升 | 告警 `takeover_rate`；允许回滚 `current` 到上一 git_sha |
| 安全链失败 | 失败池；禁止该 robot 的模型晋升 |
