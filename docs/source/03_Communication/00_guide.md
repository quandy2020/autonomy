# Communication 通信框架指南

`autolink` 是Autonomy的**分布式通信运行时**，为规划、控制、定位、地图等模块提供 Node / Channel / Service / Action 等通信原语，底层支持进程内、共享内存与 RTPS 等多种传输后端。本文档为模块总入口，以下按 **§1–§9** 顺序组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§5 架构](05_architecture.md) → [§4 使用指南](04_usage.md) |
| 模块开发 | [§6 Node 与 Channel](06_node_channel.md) → [§7 Service 与 Action](07_service_action.md) → [§4 使用指南](04_usage.md) |
| 性能调优 | [§8 调度与传输](08_scheduler_transport.md) → [§5 架构](05_architecture.md) → [§9 综述](09_survey.md) |

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 模块概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_concepts.md](03_concepts.md) | 通信概念与模型 |
| 4 | [04_usage.md](04_usage.md) | 使用指南 |
| 5 | [05_architecture.md](05_architecture.md) | 模块架构设计 |
| 6 | [06_node_channel.md](06_node_channel.md) | Node 与 Channel |
| 7 | [07_service_action.md](07_service_action.md) | Service 与 Action |
| 8 | [08_scheduler_transport.md](08_scheduler_transport.md) | 调度与传输 |
| 9 | [09_survey.md](09_survey.md) | 通信框架综述 |

<div class="nav-costmap-banner">
  <strong>autolink 在 Autonomy 栈中的位置</strong>
  <span class="nav-costmap-detail">算法模块（C++ struct）→ commsgs ToProto → autolink Writer/Reader → 跨进程 / 跨机</span>
  <span class="nav-costmap-arrow">对标 Cyber RT / ROS 2 rclcpp →</span>
</div>
