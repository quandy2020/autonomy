# Visualization 可视化指南

本文档说明如何在 Autonomy 导航栈中**查看地图、路径、传感器与状态数据**。

> **实现状态**：内置 `autonomy/visualization` 模块与 `VisualizationServer` **尚未落地**；当前推荐通过 **ROS 2 + RViz2** 或 **Foxglove Studio + foxglove_bridge** 进行可视化。纯 C++ 离线运行依赖日志与 `autonomy_nav_test` 验证。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 快速调试 | [§2 快速开始](02_quickstart.md) |
| ROS 2 用户 | [§7 RViz2](07_rviz2_ros2.md) |
| Foxglove 用户 | [§6 Foxglove](06_foxglove.md) |
| 架构/开发 | [§5 架构](05_architecture.md) → [§4 配置](04_configuration.md) |

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 三条可视化路径与现状 |
| 2 | [02_quickstart.md](02_quickstart.md) | RViz2 / Foxglove 快速接入 |
| 3 | [03_data_types.md](03_data_types.md) | commsgs 可视化消息 |
| 4 | [04_configuration.md](04_configuration.md) | visualization.lua |
| 5 | [05_architecture.md](05_architecture.md) | 当前与目标数据流 |
| 6 | [06_foxglove.md](06_foxglove.md) | Foxglove Studio |
| 7 | [07_rviz2_ros2.md](07_rviz2_ros2.md) | RViz2 与 autonomy_ros |
| 8 | [08_survey.md](08_survey.md) | 工具选型与路线图 |
