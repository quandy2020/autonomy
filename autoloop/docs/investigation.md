# Investigation（可追查）

追查目标：家居导航失败、定位丢失、任务失败，能给出 **可引用的根因卡片**（哪一窗、哪一通道、哪一 BT 节点），而不是只留一段 bag。

## 能查到什么 / 查不到什么

| 能 | 不能（需 SIL/HIL） |
|----|-------------------|
| 失败时刻前后窗口内的 odom / plan / cmd / BT / 定位质量 / 接管 | 未订阅通道的物理真值 |
| 时间共现：定位质量下降 → 随后 path 偏离 → 任务 FAILED | 传感器硬件损坏的物理因果 |
| 四元组证明「这是哪版地图/模型」 | 未打 `env_version` 的现场布局变化 |
| 安全闩是否在失败前已置位 | 采集进程去刹停机器人 |

L2 是 **时间窗 + 通道共现**。文档与 UI 不得把「相关」写成「导致」，除非 SIL 对照复现成功。

## 机上线索

触发边沿（见 [trigger_rules](trigger_rules.md)）写入 `TriggerEvent{kind, priority, mono_ns, reason}`，进入 sidecar。

| 线索 | 通道 / 信号 | 根因卡片字段 |
|------|-------------|--------------|
| 任务失败 | TaskResult / `*/feedback` FAILED | `task_id`, `result` |
| 定位降级 | localization quality 边沿 | `loc_quality`, `floor` |
| 路径偏离 | odom vs plan | `deviate_m` |
| 接管 | teleop 上升沿 | `takeover=true` |
| 急停 | 闩锁文件 | `latch_set=true` |
| BT | `/behavior_tree/log` | `bt_node`, `status` |

观察（人工或自动标）必须带 `evidence_node_id`，否则不进归因。

## 云端追查路径

```
仪表告警 / 日志 ERROR
    → trace_id | episode_id
    → trace_index + metadata_store
    → sidecar（admitted? missing_fields? 四元组?）
    → MCAP 时间轴 + 事件图
    → failure_pool 或 diagnostics
    → replay 同 seed（可选）
```

`diagnostics/` 三类：

1. **未准入**：缺四元组。先补版本源再谈训练。
2. **破损**：哈希失败、MCAP 截断。可 `RecoverMcap` / 重传。
3. **已准入仍失败**：进 `failure_pool`，挂根因卡片，供回归集。

## 家居导航失败的展现

| 现场 | 操作员看到 | 工程师落到 |
|------|------------|------------|
| 走偏 / 停住 | 任务失败 toast + episode 链接 | sidecar `task_fail` + plan/odom overlay |
| 定位丢失 | 「定位降级」+ 地图版本 | `loc_degrade` + `env_version` |
| 人接手 | 「接管」高亮 | P1，不进专家模仿除非人审放行 |
| 急停 | 闩锁保持，需人工 Clear | 安全链新条目，P0 |

效果上：**每次业务失败至少有一条可打开的 episode**；若只有日志没有 MCAP，记为数据丢失告警，不算闭环完成。
