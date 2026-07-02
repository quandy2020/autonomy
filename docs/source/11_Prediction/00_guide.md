# Prediction 轨迹预测指南

`autonomy/prediction` 是 Autonomy 的**动态障碍轨迹预测**子系统，接收 `perception` 输出的障碍/检测列表，预测其未来运动轨迹，供 `planning` 做交互式避障与速度规划。本文档为模块总入口，以下按 **§1–§9** 顺序组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§5 架构](05_architecture.md) |
| 算法研发 | [§9 综述](09_survey.md) → [§3 数学原理](03_math.md) → [§7 轨迹预测](07_trajectory_prediction.md) |
| 集成调试 | [§2 快速开始](02_quickstart.md) → [§4 使用指南](04_usage.md) → [§6 运动模型](06_motion_models.md) |

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 模块概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_math.md](03_math.md) | 数学原理 |
| 4 | [04_usage.md](04_usage.md) | 使用指南 |
| 5 | [05_architecture.md](05_architecture.md) | 模块架构设计 |
| 6 | [06_motion_models.md](06_motion_models.md) | 运动模型 |
| 7 | [07_trajectory_prediction.md](07_trajectory_prediction.md) | 轨迹预测算法 |
| 8 | [08_behavior_prediction.md](08_behavior_prediction.md) | 行为与交互预测 |
| 9 | [09_survey.md](09_survey.md) | 预测算法综述 |
