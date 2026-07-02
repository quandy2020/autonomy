# Planning 路径规划指南

`autonomy/planning` 是 Autonomy 的**全局路径规划**子系统，对齐 ROS 2 Navigation2 `nav2_planner`。本文档为模块总入口，以下按 **§1–§9** 顺序组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§5 架构](05_architecture.md) → [§4 使用指南](04_usage.md) |
| 算法研发 | [§9 综述](09_survey.md) → [§3 数学原理](03_math.md) → [§6 NavFn](06_navfn.md) / [§8 Theta*](08_theta_star.md) |
| 集成调试 | [§2 快速开始](02_quickstart.md) → [§4 使用指南](04_usage.md) → [§4.7 选型](04_usage.md#47-规划器选型) → [§4.8 排错](04_usage.md#48-故障排查) |

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 模块概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_math.md](03_math.md) | 数学原理 |
| 4 | [04_usage.md](04_usage.md) | 使用指南 |
| 5 | [05_architecture.md](05_architecture.md) | 模块架构设计 |
| 6 | [06_navfn.md](06_navfn.md) | NavFn 全局规划器 |
| 7 | [07_dijkstra.md](07_dijkstra.md) | Dijkstra 全局规划器 |
| 8 | [08_theta_star.md](08_theta_star.md) | Theta* 全局规划器 |
| 9 | [09_survey.md](09_survey.md) | 路径规划算法综述 |
