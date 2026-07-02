(tasks-overview)=
# 1. 任务概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 层级 | **导航任务层**（Navigation Task Layer） |
| 入口 | `system::Autonomy` |
| 编排 | [Navigator](../16_Navigator/01_overview.md) + 行为树（演进中） |
| 对标 | nav2_bt_navigator 的任务语义 |
| 消息 | `commsgs::nav_msgs::*Action` |

Autonomy **无独立 `autonomy/tasks/` 包**。导航任务 = `Autonomy` 对外 API + Navigator 内部编排。

### 1.2 任务与模块关系

```
用户 / Bridge / 测试工具
        │
        ▼
  system::Autonomy          ← 本章（任务 API）
        │
        ├── NavigateToPose / NavigateThroughPoses
        │
        ▼
  navigator（BT 编排）       ← 16 Navigator
        │
        ├── PlannerServer（规划）
        ├── ControllerServer（跟踪）
        └── Map / Transform
```

### 1.3 支持的任务类型

| 任务 | API | 说明 |
|------|-----|------|
| 单点导航 | `NavigateToPose()` | 从当前位姿到目标位姿 |
| 多点导航 | `NavigateThroughPoses()` | 依次经过多个航点 |
| 重规划 | `ReplanToGoal()` | 保持目标，重新计算路径 |

### 1.4 当前实现状态

| 能力 | 状态 | 说明 |
|------|------|------|
| 任务 API 定义 | ✅ | `autonomy.hpp` |
| 配置管线 | ✅ | `NavigatorOptions` / `navigator.lua` |
| BT XML | ✅ | `navigate_to_pose.xml` 等 |
| BT 引擎 + 插件 | ⏳ | 源码待恢复 |
| 直驱规划 | ✅ | `NavigateDirectToPose` → `GetPlan` |
| 完整闭环（规划+控制） | ⏳ | BT 未默认启用 |

### 1.5 与 `common::Task` 的区别

`autonomy/common/task.hpp` 中的 `Task` 是**线程池异步工作项**（Cartographer 风格），与导航「任务」无关。

### 1.6 相关文档

- [§2 快速开始](02_quickstart.md)
- [16 Navigator](../16_Navigator/index.rst)
- [05 Framework · 模块 Server](../05_Framework/05_module_servers.md)
