# 7. 预测综述

公式：[§3](03_math.md)；运动模型：[§6](06_motion_models.md)。

### 7.1 仓库现状

| 能力 | 状态 |
|------|------|
| `common/motion_model`（CV/CA、差速、静止） | — |
| 卡尔曼滤波 | `common/state_estimation` |
| `PredictionServer` 流水线 | 壳 |
| 学习型 / 博弈预测 | 未实现（不做专题页） |

### 7.2 物理模型

| 模型 | 适用 | 状态 |
|------|------|------|
| CV / CA | 行人、加减速 | `LinearMotionModel` |
| DiffDrive / Stationary | 机器人 / 静态 | — |

目标管线（未接入）：Perception → Prediction → Planning。当前导航仍用 costmap。
