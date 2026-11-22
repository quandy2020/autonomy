# 07 追查（可追查）

追查目标：家居导航失败、定位丢失、任务失败，给出 **可引用的根因卡片**（哪一窗、哪一通道、哪一 BT 节点），而不是只留一段 bag。L2 是时间窗 + 通道共现；不得把「相关」写成「导致」，除非 SIL 对照成功。

## 能 / 不能

| 能 | 不能（需 SIL/HIL） |
|----|-------------------|
| 失败时刻前后窗口内 odom / plan / cmd / BT / 定位质量 / 接管 | 未订阅通道的物理真值 |
| 时间共现：定位质量下降 → path 偏离 → 任务 FAILED | 传感器硬件损坏的物理因果 |
| 四元组证明地图 / 模型版本 | 未打 `env_version` 的现场布局变化 |
| 安全闩是否在失败前已置位 | 采集进程去刹停机器人 |

## 机上线索

触发边沿写入 `TriggerEvent{kind, priority, mono_ns, reason}`，进入 sidecar。

| 线索 | 通道 / 信号 | 根因卡片字段 |
|------|-------------|--------------|
| 任务失败 | TaskResult / `*/feedback` FAILED | `task_id`, `result` |
| 定位降级 | localization quality 边沿 | `loc_quality`, `floor` |
| 路径偏离 | odom vs plan | `deviate_m` |
| 接管 | teleop 上升沿 | `takeover=true` |
| 急停 | 闩锁文件 | `latch_set=true` |
| BT | `/behavior_tree/log` | `bt_node`, `status` |
| 代价尖峰 | `cost_channel` 非空时 | `cost_sigma` |

观察必须带 `evidence_node_id`，否则不进 `AcceptObservation` / L2。recorder `RankRootCauses` 产出卡片；`AttributeBtFailure` 填 BT 节点。

## 云端路径

```text
仪表告警 / 日志 ERROR
    → trace_id | episode_id
    → trace_index + metadata_store
    → sidecar（admitted? missing_fields? 四元组?）
    → MCAP 时间轴 + 事件图
    → failure_pool 或 diagnostics
    → replay 同 seed（可选）
```

`diagnostics/` 三类：未准入（补版本源）、破损（`RecoverMcap` / 重传）、已准入仍失败（挂卡片进 `failure_pool` / `regression_set`）。

## 家居失败展现

| 现场 | 操作员看到 | 工程师落到 |
|------|------------|------------|
| 走偏 / 停住 | 任务失败 toast + episode 链接 | sidecar `task_fail` + plan/odom overlay |
| 定位丢失 | 「定位降级」+ 地图版本 | `loc_degrade` + `env_version` |
| 人接手 | 「接管」高亮 | P1，不进专家模仿除非人审放行 |
| 急停 | 闩锁保持，需人工 Clear | 安全链新条目，P0 |

**验收**：每次业务失败至少有一条可打开的 episode；只有日志没有 MCAP → 告警 `data_loss`，不算闭环完成。
