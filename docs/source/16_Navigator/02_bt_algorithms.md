# 2. BT 算法总览

改 XML 结构、加 Recovery，或对比单点 / 多点 / 直驱。引擎见 [§3](03_bt_engine.md)，单点树 [§4](04_navigate_to_pose.md)，插件 [§5](05_bt_plugins.md)。

行为树在 `autonomy/task/` 由 TaskServer 加载 XML 与 BT 插件 `.so`，每 tick 驱动 planning/control Action。先选形态（单点 `navigate_to_pose`、多点 `navigate_through_poses`、或调试直驱），再按「规划 → 平滑 → 验路 → FollowPath」流水线挂 Recovery（清图、备份、旋转等）。控制节点语义（Sequence/PipelineSequence、ReactiveFallback、RecoveryNode）决定失败时是否重试或切局部生存。

### 核心组件

| 组件 | 职责 |
|------|------|
| BtRunner / TaskApp | Goal ↔ tick 循环 |
| 插件 `.so` | 52 量级节点（以 conf 列表为准） |
| XML | 组合 control/decorator/action |

### 三种运行形态

| 形态 | 树/路径 | 特点 |
|------|---------|------|
| 单点 | `navigate_to_pose.xml` | GoalReached + LocalSurvival |
| 多点 | `navigate_through_poses.xml` | 航点序列，更高频规划 |
| 直驱 | 无完整 BT | GetPlan 调试 |

### 常用控制节点

| 节点 | 语义 |
|------|------|
| `Sequence` / `PipelineSequence` | 顺序；Pipeline 允许前子 RUNNING 时推进 |
| `ReactiveFallback` | 每 tick 从首子重试 |
| `RecoveryNode` | 主失败 → 恢复子 → 重试 |
| `RoundRobin` | 轮转子节点（局部生存） |
| `RateController` | 限频（规划 5–10 Hz） |

### 标准流水线

```text
Selector(planner/controller/…) → ComputePath → SmoothPath → IsPathValid → FollowPath
失败 → ClearCostmap + BackUp + Spin（Recovery）
```
