# Diagnostics

未准入、破哈希、上传中断、混版进入诊断通道，不进训练。追查步骤见 [investigation](investigation.md)。

| 入口 | 作用 |
|------|------|
| `trace_index` | `trace_id` / `episode_id` / `git_sha` → 归档路径 |
| `metadata_store` | sidecar 字段检索、人审状态 |
| `replay_tools` | DualClock 回放（SIL） |
| `regression_runner` | 固定 seed 跑 `regression_set` |
| `monitor_bridge` | 机上 health / latch → 云端仪表 |

`missing_fields` 与 recorder 诊断队列对齐。告警 YAML：`alerts/error_density.yaml`、`data_loss.yaml`、`safety_event.yaml`。
