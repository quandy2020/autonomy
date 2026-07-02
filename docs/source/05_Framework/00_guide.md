# Autonomy 框架指南

本文档描述 **`libautonomy` 应用框架**：如何将 map、planning、control、navigator 等模块装配为可运行系统。底层通信由 **Autolink** 提供，详见 [03 Communication](../03_Communication/00_guide.md)。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§3 架构](03_architecture.md) |
| 模块开发 | [§5 模块 Server](05_module_servers.md) → [§6 插件系统](06_plugin_system.md) |
| 通信集成 | [03 Communication](../03_Communication/00_guide.md) → [§7 消息集成](07_commsgs_integration.md) |
| 扩展定制 | [§8 综述](08_survey.md) → [§4 配置管线](04_configuration.md) |

<div class="nav-costmap-banner">
  <strong>Framework vs Communication</strong>
  <span class="nav-costmap-detail">Framework = 模块装配 + 配置 + Server；Communication = Autolink Node/Channel/Action API</span>
  <span class="nav-costmap-arrow">二者互补，不重复 →</span>
</div>

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 框架概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_architecture.md](03_architecture.md) | 框架架构 |
| 4 | [04_configuration.md](04_configuration.md) | 配置管线 |
| 5 | [05_module_servers.md](05_module_servers.md) | 模块 Server |
| 6 | [06_plugin_system.md](06_plugin_system.md) | 插件系统 |
| 7 | [07_commsgs_integration.md](07_commsgs_integration.md) | 消息集成 |
| 8 | [08_survey.md](08_survey.md) | 框架综述 |
