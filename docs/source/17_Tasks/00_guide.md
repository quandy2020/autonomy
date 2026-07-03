# Tasks 导航任务指南

本文档描述 Autonomy 的**导航任务层**：用户如何下发目标、任务如何被编排与执行。

> **与 Navigator 的关系**：历史上存在独立 `tasks` 模块，已合并进 `navigator`。Tasks 文档侧重**用户任务 API 与语义**；BT 实现细节见 [16 Navigator](../16_Navigator/00_guide.md)。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 应用开发者 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§3 任务类型](03_task_types.md) |
| 配置调参 | [§4 任务配置](04_configuration.md) → [§6 执行模式](06_execution_modes.md) |
| 架构理解 | [§5 任务架构](05_architecture.md) → [16 Navigator](../16_Navigator/01_architecture.md) |

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 任务层定位 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速下发导航目标 |
| 3 | [03_task_types.md](03_task_types.md) | NavigateToPose 等任务类型 |
| 4 | [04_configuration.md](04_configuration.md) | navigator.lua 配置 |
| 5 | [05_architecture.md](05_architecture.md) | 任务执行架构 |
| 6 | [06_execution_modes.md](06_execution_modes.md) | BT 模式 vs 直驱模式 |
| 7 | [07_api_reference.md](07_api_reference.md) | Autonomy API 参考 |
| 8 | [08_survey.md](08_survey.md) | 能力矩阵与演进 |
