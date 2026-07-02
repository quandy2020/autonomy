# Autonomy 入门指南

**Autonomy** 是面向移动机器人的自主导航软件框架：模块化 C++ 库、Lua 配置、行为树任务编排、完整 2D 导航栈，并可与 ROS 2 消息生态及 gRPC 桥接层互操作。本文档为全书总入口，按 **§1–§8** 组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 首次接触 | [§1 概览](01_overview.md) → [§2 快速上手](02_quickstart.md) → [§4 导航栈](04_navigation_stack.md) |
| 系统集成 | [§3 系统架构](03_system_architecture.md) → [§5 工程结构](05_project_layout.md) → [§6 生态集成](06_ecosystem.md) |
| 文档查阅 | [§7 文档导读](07_documentation_guide.md) → 各模块 `00_guide.md` |
| 贡献者 | [§5 工程结构](05_project_layout.md) → [§8 版本与路线](08_roadmap.md) |

<div class="nav-costmap-banner">
  <strong>Autonomy 一句话定位</strong>
  <span class="nav-costmap-detail">独立运行的 C++17 导航框架 · 对齐 Navigation2 语义 · 可选 ROS 2 / gRPC 互操作</span>
  <span class="nav-costmap-arrow">从 §2 克隆编译 → §4 理解导航栈 → 各模块深入 →</span>
</div>

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 项目概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速上手 |
| 3 | [03_system_architecture.md](03_system_architecture.md) | 系统架构 |
| 4 | [04_navigation_stack.md](04_navigation_stack.md) | 导航栈全景 |
| 5 | [05_project_layout.md](05_project_layout.md) | 工程结构 |
| 6 | [06_ecosystem.md](06_ecosystem.md) | 生态集成 |
| 7 | [07_documentation_guide.md](07_documentation_guide.md) | 文档导读 |
| 8 | [08_roadmap.md](08_roadmap.md) | 版本与路线 |
