# 2. BT 算法总览

专题索引；引擎 [§3](03_bt_engine.md)、单点树 [§4](04_navigate_to_pose.md)、插件 [§5](05_bt_plugins.md)。

### 2.1 组件

| 组件 | 职责 |
|------|------|
| `BtEngine` | 加载插件、解析 XML |
| `BtActionServer` | Goal ↔ tick 循环 |
| `BtContext` | planner / controller / TF / options |
| 插件 `.so` | 52 节点 |

### 2.2 模式对比

| 模式 | 树 | 特点 |
|------|-----|------|
| 单点 | `navigate_to_pose.xml` | GoalReached + 局部生存 |
| 多点 | `navigate_through_poses.xml` | 航点序列，10 Hz 规划 |
| 直驱 | 无 BT | GetPlan 调试 |

### 2.3 控制节点

| 节点 | 语义 |
|------|------|
| `Sequence` / `PipelineSequence` | 顺序；Pipeline 允许前子 RUNNING 时推进 |
| `ReactiveFallback` | 每 tick 重检首子 |
| `RecoveryNode` | 主分支失败 → 恢复 → 重试 |
| `RoundRobin` | 轮转子节点 |
| `RateController` | 限频 tick 子树 |

### 2.4 标准流水线

```text
Selector×3 → ComputePath → SmoothPath → IsPathValid → FollowPath
失败 → ClearCostmap + BackUp + Spin（Recovery）
```
