# Commsgs 通信消息指南

`autonomy/commsgs` 是 Autonomy 的**通信消息**子系统，在 ROS 2 `common_msgs` 语义基础上，采用 **C++ struct + Protocol Buffers** 双层设计，供规划、地图、控制、定位、可视化等模块共享。本文档为模块总入口，以下按 **§1–§9** 顺序组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§5 架构](05_architecture.md) → [§4 使用指南](04_usage.md) |
| 消息扩展开发 | [§3 Schema](03_schema.md) → [§5 架构](05_architecture.md) → [§6 核心消息](06_core_msgs.md) → [§4.6 扩展](04_usage.md#46-扩展自定义消息) |
| 集成调试 | [§2 快速开始](02_quickstart.md) → [§4 使用指南](04_usage.md) → [§4.8 排错](04_usage.md#48-故障排查) → [§9 综述](09_survey.md) |

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 模块概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_schema.md](03_schema.md) | 消息模型与 Schema 设计 |
| 4 | [04_usage.md](04_usage.md) | 使用指南 |
| 5 | [05_architecture.md](05_architecture.md) | 模块架构设计 |
| 6 | [06_core_msgs.md](06_core_msgs.md) | 核心消息类型 |
| 7 | [07_sensor_map_msgs.md](07_sensor_map_msgs.md) | 传感器与地图消息 |
| 8 | [08_nav_planning_msgs.md](08_nav_planning_msgs.md) | 导航与规划消息 |
| 9 | [09_survey.md](09_survey.md) | 消息体系综述 |

<div class="nav-costmap-banner">
  <strong>commsgs 在导航栈中的位置</strong>
  <span class="nav-costmap-detail">各算法模块内部使用 C++ struct → Autolink 边界 ToProto/FromProto → 网络传输</span>
  <span class="nav-costmap-arrow">对齐 ROS 2 common_msgs 语义 →</span>
</div>
