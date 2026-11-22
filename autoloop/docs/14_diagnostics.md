# 14 诊断通道

未准入、破哈希、上传中断、混版进入诊断通道，不进训练。追查步骤见 [07](07_investigation.md)。

## 入口

| 模块 | 作用 |
|------|------|
| `trace_index` | `trace_id` / `episode_id` / `git_sha` → 归档路径 |
| `metadata_store` | sidecar 字段检索、人审状态 |
| `replay_tools` | DualClock 回放、裁剪、对比 |
| `regression_runner` | 固定 seed 跑 `regression_set` |
| `monitor_bridge` | 机上 health / latch → 云端仪表 |
| `log_ingestion` | 解析 glog 前缀入索引 |
| `telemetry_gate` | 混版 / 指标准入 |

`missing_fields` 与 recorder `diagnostics/` 对齐。告警：`alerts/error_density.yaml`、`data_loss.yaml`、`safety_event.yaml`。

## 分流

| 入口条件 | 目录 / 池 | 可否训练 |
|----------|-----------|----------|
| 四元组缺字段 | diagnostics | 否 |
| `content_sha256` 不匹配 | diagnostics + `data_loss` | 否 |
| MCAP 截断 | diagnostics，可 Recover | 否 |
| 已准入任务失败 | `failure_pool` | 否（模仿）；可进回归 |
| 混版 | telemetry 标记 | 不当新模型正例 |
| 上传中断 | staging 书签，P0 优先 | 待重传 |

## 告警 payload

必须含 `trace_id`、`robot_id`、`episode_id`（若已有）。无 MCAP 的失败记 `data_loss`，不算闭环完成。
