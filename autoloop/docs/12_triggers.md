# 12 触发规则

命中后拷贝 ring / rolling 窗口落盘。采集路径不得发布控制话题。测试全量用 `--full-dump`，不靠触发截断。

## data_agent

配置：`robot/config/data_agent/{conservative,aggressive,minimal}.yaml`。

| 文件 | `window_sec` | `max_samples` | `priority_floor` |
|------|--------------|---------------|------------------|
| conservative | 2.0 | 2048 | 1 |
| aggressive | 4.0 | 8192 | 3 |
| minimal | 1.0 | 512 | 0 |

`TriggerKind`：`kEstop` / `kTaskFail` / `kTakeover` / `kLocDegrade` / `kManual` / `kPeriodic`。`DataAgentComponent` 默认注册 takeover、task_fail、estop 为 P0。`ManualSnapshot` 走 `trigger_snapshot.sh`。热路径 `Snapshot()` 拷贝窗口，不用 `TakeAll`。

## recorder

`TriggerEngine` 在订阅回调与 `SpinOnce` 超时轮询中产生事件：急停闩、BT recover / 任务根 FAILURE、`*/feedback` FAILED、定位质量边沿、路径偏离、可选 cost、teleop、通道 `timeout_sec>0`。episode 开闭见 [recorder 02](recorder/02_design.md)。滚动窗默认 `buffer_sec=30`。

## 优先级与去向

| 规则 | 优先级 | 上传 | 训练 |
|------|--------|------|------|
| estop / task_fail | P0 | 立即 | 失败池，不进模仿正例 |
| takeover | P0/P1 | 是 | 默认不进专家集 |
| loc_degrade | P1 | 是 | 可进回归 |
| path_deviate / manual | P2 | 抽样 | 可选人审 |
| periodic / 成功 | P3 | 分层抽样 | 可进 success_pool |

四元组不齐无论 P 级均 diagnostics。`priority_floor` 丢弃低于下限的触发（minimal 保留全部）。
