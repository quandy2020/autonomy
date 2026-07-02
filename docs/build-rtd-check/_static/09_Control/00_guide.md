# Control 运动控制指南

`autonomy/control` 是 Autonomy 的**局部运动控制**子系统，对齐 ROS 2 Navigation2 `nav2_controller`。接收全局路径，以固定频率计算并发布 `cmd_vel`，直至目标到达。本文档为模块总入口，以下按 **§1–§9** 顺序组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§5 架构](05_architecture.md) → [§4 使用指南](04_usage.md) |
| 算法研发 | [§9 综述](09_survey.md) → [§3 数学原理](03_math.md) → [§8 局部控制器](08_controller_algorithms.md) |
| 集成调试 | [§2 快速开始](02_quickstart.md) → [§4 使用指南](04_usage.md) → [§6 检查器](06_checkers.md) → [§4.8 排错](04_usage.md#48-故障排查) |

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 模块概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_math.md](03_math.md) | 数学原理 |
| 4 | [04_usage.md](04_usage.md) | 使用指南 |
| 5 | [05_architecture.md](05_architecture.md) | 模块架构设计 |
| 6 | [06_checkers.md](06_checkers.md) | 目标与进度检查器 |
| 7 | [07_velocity_smoother.md](07_velocity_smoother.md) | 速度平滑器 |
| 8 | [08_controller_algorithms.md](08_controller_algorithms.md) | 局部控制器算法 |
| 9 | [09_survey.md](09_survey.md) | 运动控制算法综述 |
