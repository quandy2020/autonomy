(framework-overview)=
# 1. 框架概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 层级 | **应用框架**（Application Framework） |
| 核心库 | `libautonomy.so` |
| 职责 | 模块装配、配置加载、Server 生命周期、导航 API |
| 通信层 | [Autolink](../03_Communication/01_overview.md)（独立子项目） |
| 消息层 | [commsgs](../14_Commsgs/01_overview.md) |
| 对标 | Navigation2 栈组合 + Cyber RT 组件思想 |

**Framework** 回答「各模块如何组织、配置与启动」；**Communication** 回答「进程间如何收发消息」。

### 1.2 三层模型

```
┌─────────────────────────────────────────┐
│  Application Framework (本章)            │
│  system::Autonomy · Server · 插件 · 配置  │
└──────────────────┬──────────────────────┘
                   │
┌──────────────────▼──────────────────────┐
│  commsgs — 消息类型（C++ struct + proto） │
└──────────────────┬──────────────────────┘
                   │
┌──────────────────▼──────────────────────┐
│  autolink — Node / Channel / Service / Action │
└───────────────────────────────────────────┘
```

### 1.3 核心组件

| 组件 | 路径 | 职责 |
|------|------|------|
| `system::Autonomy` | `autonomy/system/` | 顶层入口，持有各 Server |
| `MapServer` | `autonomy/map/` | 静态地图、发布 OccupancyGrid |
| `PlannerServer` | `autonomy/planning/` | 全局规划、costmap |
| `ControllerServer` | `autonomy/control/` | 局部控制、里程计 |
| `navigator` | `autonomy/navigator/` | BT **导航编排** |
| `transform` | `autonomy/transform/` | TF 缓冲与静态外参 |
| `bridge` | `autonomy/bridge/` | gRPC 等外部桥接 |

### 1.4 设计原则

1. **Server 模式**：各子系统以 `*Server` 类对外，内部可持有 autolink Node
2. **配置驱动**：Lua → Protobuf → `*Options` → Server 构造
3. **插件化算法**：规划器、控制器、BT 节点以 `.so` 动态加载
4. **Nav2 语义对齐**：接口与话题/Action 命名尽量与 Navigation2 一致
5. **可独立运行**：不强制 ROS 2；`commsgs` 提供消息兼容

### 1.5 与旧文档的关系

原 `autonomy_framework_and_communication.md` 中的 **Autolink API 详解** 已迁移至 [03 Communication](../03_Communication/00_guide.md)。本章聚焦 **Autonomy 应用层**。

### 1.6 相关文档

- [§2 快速开始](02_quickstart.md)
- [01 Instructions · 系统架构](../01_Instructions/03_system_architecture.md)
