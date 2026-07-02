# Navigator 导航编排指南

`autonomy/navigator` 是 Autonomy 的**导航编排**子系统，对齐 ROS 2 Navigation2 `nav2_bt_navigator`。通过行为树（Behavior Tree, BT）协调 `planning`、`control`、`map`、`localization` 等模块，完成单点导航与多点巡航。本文档为模块总入口，以下按 **§1–§9** 顺序组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§5 架构](05_architecture.md) → [§4 使用指南](04_usage.md) |
| BT 定制 | [§7 单点导航 BT](07_navigate_to_pose.md) → [§8 BT 插件](08_bt_plugins.md) → [§6 行为树引擎](06_bt_engine.md) |
| 算法研发 | [§9 综述](09_survey.md) → [§3 数学原理](03_math.md) → [§7 单点导航 BT](07_navigate_to_pose.md) |
| 集成调试 | [§2 快速开始](02_quickstart.md) → [§4 使用指南](04_usage.md) → [§4.10 排错](04_usage.md#410-故障排查) |

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 模块概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_math.md](03_math.md) | 数学原理 |
| 4 | [04_usage.md](04_usage.md) | 使用指南 |
| 5 | [05_architecture.md](05_architecture.md) | 模块架构设计 |
| 6 | [06_bt_engine.md](06_bt_engine.md) | 行为树引擎 |
| 7 | [07_navigate_to_pose.md](07_navigate_to_pose.md) | 单点导航 BT |
| 8 | [08_bt_plugins.md](08_bt_plugins.md) | BT 插件节点 |
| 9 | [09_survey.md](09_survey.md) | **导航编排**综述 |
