# 2. 导航编排 BT 算法

Autonomy `navigator` 通过 **BehaviorTree.CPP** 与 52 个插件节点编排规划、控制与恢复。本文是 **§3–§5 专题的索引与对比**；形式化见 [§0.3](00_guide.md#03-问题形式化)，架构见 [§1](01_architecture.md)，谱系与选型见 [§6 综述](06_survey.md)。

---

## 2.1 组件一览

| 组件 | 文档 | 职责 | 状态 |
|------|------|------|------|
| `BtEngine` | [§3 引擎](03_bt_engine.md) | 插件加载、XML 解析、工厂 | ⏳ |
| `BtActionServer` | [§3](03_bt_engine.md) | Action ↔ BT tick 循环 | ⏳ |
| `navigate_to_pose.xml` | [§4 单点 BT](04_navigate_to_pose.md) | 规划→跟踪→恢复 + 局部生存 | ✅ XML |
| `navigate_through_poses.xml` | [§5 §5.7](05_bt_plugins.md#57-多点导航树-navigate_through_posesxml) | 多点巡航 + 10 Hz 重规划 | ✅ XML |
| BT 插件（52） | [§5 插件](05_bt_plugins.md) | Action / Condition / Control / Decorator | ⏳ |

**继承关系**

```
BehaviorTreeNavigator<ActionT>
├── NavigateToPoseNavigator      ← navigate_to_pose.xml
└── NavigateThroughPosesNavigator ← navigate_through_poses.xml
```

---

## 2.2 编排模式对比

| 维度 | 直驱模式 | BT 单点 | BT 多点 |
|------|----------|---------|---------|
| 入口 | `NavigateDirectToPose` | `navigate_to_pose.xml` | `navigate_through_poses.xml` |
| 规划 | 单次 `GetPlan` | `RateController` 5 Hz | 10 Hz 贯穿航点 |
| 跟踪 | 无 | `FollowPath` | `FollowPath` |
| 恢复 | 无 | `RecoveryNode` ×8 + 局部生存 | `RecoveryNode` ×6 |
| TF 丢失 | 立即失败 | 局部生存模式 | nav2 同型（无局部生存） |
| 典型场景 | 规划调试 | 标准室内导航 | 仓库巡检 |

---

## 2.3 控制节点语义

| 节点 | 代数语义 | Autonomy 用途 |
|------|----------|---------------|
| `Sequence` | $N_1 \land \cdots \land N_m$ | 顺序执行 |
| `Fallback` | $N_1 \lor \cdots \lor N_m$ | 恢复备选 |
| `ReactiveFallback` | 每 tick 重求 $\lor$ | `GoalReached` 优先 |
| `PipelineSequence` | 流水线激活 | 规划→平滑→跟踪 |
| `RecoveryNode` | $\mathrm{retry}(M, R, K)$ | SafeNavigate |
| `RateController` | 频率限制子节点 | 5/10 Hz 重规划 |

详述见 [§3.7](03_bt_engine.md#37-控制节点语义) 与 [§6.5.1](06_survey.md#651-行为树导航理论与优势)。

---

## 2.4 标准流水线

$$
\mathrm{Plan} \xrightarrow{\mathrm{Smooth}} \mathrm{Validate} \xrightarrow{\mathrm{Follow}} \mathrm{Goal}
$$

| 阶段 | BT 节点 | 子系统 |
|------|---------|--------|
| 规划 | `ComputePathToPose` / `ComputePathThroughPoses` | `PlannerServer` |
| 平滑 | `SmoothPath` | Smoother |
| 验证 | `IsPathValid` | `PlannerServer` |
| 跟踪 | `FollowPath` | `ControllerServer` |
| 恢复 | `ClearEntireCostmap` + `BackUp` + `Spin` | Map + Control |

单点树在流水线外包裹 `ReactiveFallback`（TF 分支）与顶层 `RecoveryNode`。XML 逐层见 [§4](04_navigate_to_pose.md)。

---

## 2.5 选型速查

| 场景 | 推荐 | 关键配置 |
|------|------|----------|
| 规划算法调试 | 直驱 | `use_bt_navigation = false` |
| 标准室内 | BT 单点默认 XML | `navigate_to_pose.xml` |
| 定位不稳定 | BT 单点 + 增大超时 | `local_survival_timeout = 180` |
| 动态拥挤 | BT + 10 Hz 规划 | `RateController hz=10` |
| 窄通道 | BT + Theta* | `planner_selector` |
| 仓库巡检 | BT 多点 | `navigate_through_poses.xml` |

完整矩阵见 [§6.6](06_survey.md#66-autonomy-编排选型)。

---

## 2.6 扩展阅读

| § | 文档 | 内容 |
|---|------|------|
| 3 | [行为树引擎](03_bt_engine.md) | BtEngine、tick 循环、黑板 |
| 4 | [单点导航 BT](04_navigate_to_pose.md) | `navigate_to_pose.xml` 逐层 |
| 5 | [BT 插件](05_bt_plugins.md) | 52 节点目录 + 多点树 |
| 6 | [综述](06_survey.md) | 历史、分类、Nav2 对照 |
