(tasks-overview)=
# 1. 任务概览

| 维度 | 说明 |
|------|------|
| 入口 | `autonomy.task`（`TaskServer`） |
| 编排 | [Navigator / BT](../16_Navigator/00_guide.md) |
| 消息 | `commsgs::nav_msgs::*Action` |

```text
Bridge / Action Client → TaskServer → Navigator/BT → planning/control/map
```

| 任务 | 说明 |
|------|------|
| 单点 / 多点 | NavigateToPose / ThroughPoses |
| 建图 / 遥操等 | `autonomy/task/` 子模块 |

| 能力 | 状态 |
|------|------|
| TaskServer / Options / BT XML | — |
| BT 引擎 + Bridge 全量 | — |

注意：`common::Task` 是线程池工作项，与导航任务无关。
