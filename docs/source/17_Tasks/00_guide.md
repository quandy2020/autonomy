(tasks-guide)=
# 0. Tasks 指南

## 背景

Bridge 或 Action 发了目标却不动、取消不掉、搜文档搜到 `common::Task`——导航 **任务层** 和线程池里的 `common::Task` 不是一回事。本章讲 TaskServer 接收目标、任务类型与配置；BT 内部在 [16 Navigator](../16_Navigator/00_guide.md)，实现均在 `autonomy/task/`。

## 思路

```text
Bridge / Action Client → TaskServer → Navigator/BT → planning / control / map
```

- **入口进程是 `autonomy.task`** — Bridge 只是上游之一。  
- **任务类型决定 BT 与下游** — NavigateToPose / ThroughPoses 等见 [03 任务类型](03_task_types.md)。  
- **帧与默认 planner/controller id** 与 08/09/16 共用一套约定。  
- 编排实现目录：`autonomy/task/`（含 `navigation/` 子树）。


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 概览](01_overview.md) | TaskServer 与 Navigator 关系 |
| [02 快速开始](02_quickstart.md) | 最短发令 |
| [03 任务类型](03_task_types.md) | NavigateToPose / ThroughPoses 等 |
| [04 配置](04_configuration.md) | Options、XML、id |
| [05 架构](05_architecture.md) | 模块边界 |
| [06 执行模式](06_execution_modes.md) | 同步/异步、取消 |
| [07 API](07_api_reference.md) | 接口速查 |

**单点导航** → [02](02_quickstart.md) + [03](03_task_types.md)。  
**改树逻辑** → [16 Navigator](../16_Navigator/00_guide.md)。  
**云端/手机发令** → [15 Bridge](../15_Bridge/00_guide.md)。

### 2. 动手时记住

1. Action 成功但不动 — 先确认 planning/control 进程与 costmap/TF，再改 BT。  
2. 没有 **`CreateAutonomy`** 式一键工厂 — 仍走 launch + 多进程。

### 3. 常见坑

| 现象 | 先查 |
|------|------|
| Action 成功但不动 | planning/control；costmap/TF |
| 取消不掉 | [06 执行模式](06_execution_modes.md) 与 BT 取消点 |
| 搜到 common::Task | 看错命名空间 |
