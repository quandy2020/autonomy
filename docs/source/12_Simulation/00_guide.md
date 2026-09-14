# Simulation 仿真指南

Autonomy 的**仿真**能力分布在车辆抽象层、外部 ROS/Gazebo 包与配置占位中，用于在无真实硬件时验证导航栈。进程内 `autonomy_nav_test` 已移除；端到端请用多进程栈 + 外部仿真或 Bridge 发令。本文档为模块总入口，以下按 **§1–§8** 顺序组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§6 Gazebo ROS](07_gazebo_ros.md) |
| 算法研发 | [§3 数学原理](03_math.md) → [§7 Vehicle Stage](08_vehicle_stage.md) |
| ROS/Gazebo 集成 | [§6 Gazebo ROS](07_gazebo_ros.md) → [§4 使用指南](04_usage.md) |
| 架构设计 | [§5 架构](05_architecture.md) → [§7 Vehicle Stage](08_vehicle_stage.md) |

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 模块概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_math.md](03_math.md) | 数学原理 |
| 4 | [04_usage.md](04_usage.md) | 使用指南 |
| 5 | [05_architecture.md](05_architecture.md) | 模块架构设计 |
| 6 | [07_gazebo_ros.md](07_gazebo_ros.md) | Gazebo 与 ROS 集成 |
| 7 | [08_vehicle_stage.md](08_vehicle_stage.md) | 车辆抽象与 Stage |
| 8 | [09_survey.md](09_survey.md) | 仿真技术综述 |
