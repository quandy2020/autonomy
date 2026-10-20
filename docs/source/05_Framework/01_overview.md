(framework-overview)=
# 1. 框架概览

## 1.1 定位

| 维度 | 说明 |
|------|------|
| 层级 | 应用框架 |
| 职责 | 模块装配、配置加载、Server 生命周期、进程入口 |
| 通信 | [Autolink](../03_Communication/01_architecture.md) |
| 消息 | [commsgs](../14_Commsgs/01_overview.md) |

Framework 回答「如何组织、配置与启动」；Communication 回答「如何收发」。

## 1.2 三层

```text
Application Framework（各 *Server · 插件 · CreateOptions）
│
commsgs（C++ struct + proto）
│
autolink（Node / Channel / Service / Action）
```

## 1.3 核心组件

| 组件 | 路径 | 职责 |
|------|------|------|
| 模块进程 | `autonomy/*/*_main.cpp` | 独立入口 |
| `CreateOptions` | `autonomy/system/options.*` | 共享 `AutonomyOptions` |
| `MapServer` | `autonomy/map/` | 静态地图、OccupancyGrid |
| `PlannerServer` | `autonomy/planning/` | 全局规划、costmap |
| `ControllerServer` | `autonomy/control/` | 局部控制、`cmd_vel` |
| `task` | `autonomy/task/` | BT 编排 |
| `transform` | `autonomy/transform/` | TF |
| `bridge` | `autonomy/bridge/` | gRPC |

进程内 `CreateAutonomy` **已移除**；用 `autolink_launch autonomy.launch`。
