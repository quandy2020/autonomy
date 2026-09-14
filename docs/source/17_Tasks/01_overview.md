(tasks-overview)=
# 1. 任务概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 层级 | **导航任务层**（Navigation Task Layer） |
| 入口 | `autonomy.task`（`TaskServer`） |
| 编排 | [Navigator / BT](../16_Navigator/00_guide.md)（演进中） |
| 对标 | nav2_bt_navigator 的任务语义 |
| 消息 | `commsgs::nav_msgs::*Action` |

导航任务由 **TaskServer** 在独立进程中运行；进程内 `system::Autonomy` 聚合入口 **已移除**。

### 1.2 任务与模块关系

```
用户 / Bridge / Action Client
        │
        ▼
  autonomy.task (TaskServer)   ← 本章（任务进程）
        │
        ├── 导航 / 建图 / 遥操等任务类型
        │
        ▼
  planning / control / map …   ← 多进程 IPC
```

### 1.3 支持的任务类型

| 任务 | 说明 |
|------|------|
| 单点导航 | NavigateToPose 语义（BT / Action） |
| 多点导航 | NavigateThroughPoses |
| 建图 / 遥操 / 跟踪 | 见 `autonomy/task/` 子模块 |

具体 Action / 提交 API 以 `TaskServer` 与 Bridge stub 为准。

### 1.4 当前实现状态

| 能力 | 状态 | 说明 |
|------|------|------|
| `TaskServer` 进程 | ✅ | `task_main.cpp` |
| NavigatorOptions conf | ✅ | `navigator.pb.txt` 等 |
| BT XML | ✅ | `autonomy/task/conf/behavior_tree/` |
| BT 引擎 + 插件 | ⏳ | 持续演进 |
| Bridge 发令 | ⏳ | 部分 stub |

### 1.5 与 `common::Task` 的区别

`autonomy/common/task.hpp` 中的 `Task` 是**线程池异步工作项**（Cartographer 风格），与导航「任务」无关。

### 1.6 相关文档

- [§2 快速开始](02_quickstart.md)
- [16 Navigator](../16_Navigator/index.rst)
- [04 Running · 多进程栈](../04_Running/03_autonomy_process.md)
