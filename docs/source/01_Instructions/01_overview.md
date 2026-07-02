(instructions-overview)=
# 1. 项目概览

![Autonomy 系统分层架构](./images/autonomy_architecture.png)

### 1.1 是什么

**Autonomy**（`libautonomy`）是一套**自主移动机器人软件框架**，核心用 **CMake + C++17** 构建，提供：

- 2D 导航栈：代价地图、全局规划、局部控制、行为树编排
- 定位与建图：视觉 SLAM（Atlas）、地图服务
- 通信运行时：Autolink（Node / Channel / Service / Action）
- 可选桥接：gRPC、ROS 2 兼容消息（`commsgs`）

设计目标：**可独立运行**（不强制依赖 ROS 2），同时保持与 Navigation2 等生态的语义对齐，降低迁移与集成成本。

### 1.2 设计原则

| 原则 | 说明 |
|------|------|
| 模块化 | 各子系统以独立库形式组织，通过接口与配置解耦 |
| 插件化 | 规划器、控制器、BT 节点等以插件动态加载 |
| 配置驱动 | Lua → Protobuf 统一配置管线 |
| Nav2 对齐 | 接口语义、BT 结构、代价地图用法对标 Navigation2 |
| 跨平台 | x86-64 / ARM64；推荐 Ubuntu 22.04 |

### 1.3 与 ROS 2 的关系

| 维度 | Autonomy | ROS 2 |
|------|----------|-------|
| 运行时 | Autolink（可进程内/共享内存/RTPS） | rclcpp / DDS |
| 是否必须 ROS | **否**，核心可独立运行 | 是 |
| 消息类型 | `commsgs`（与 ROS 消息结构兼容） | 标准 `*_msgs` |
| 导航栈对标 | planning / control / navigator | nav2_* |
| 可视化 | Bridge / 外部工具 | RViz2、Foxglove 等 |

Autonomy **不是 ROS 2 的替代品**，而是可独立部署、并按需与 ROS 2 工具链互操作的框架。详见 [§6 生态集成](06_ecosystem.md)。

### 1.4 核心能力一览

| 领域 | 模块 | 状态 |
|------|------|------|
| 通信 | `autolink` | ✅ 运行时完整 |
| 定位 | `localization`（Atlas VSLAM） | ✅ Atlas 已实现 |
| 地图 | `map`（costmap_2d / grid_map） | ✅ |
| 规划 | `planning`（NavFn / Dijkstra / Theta\*） | ✅ |
| 控制 | `control` | ⏳ 骨架 + Checker，主循环待完成 |
| 编排 | `navigator`（行为树） | ⏳ 配置/XML 就绪，BT 栈待恢复 |
| 桥接 | `bridge`（gRPC） | ⏳ 部分实现 |
| 系统 | `system::Autonomy` | ✅ 统一入口 |

### 1.5 适用场景

- 室内/结构化环境的移动机器人导航研发
- 需要脱离 ROS 2 运行时的嵌入式或车载部署
- 基于 Navigation2 经验、希望迁移到统一 C++ 框架的团队
- 视觉 SLAM + 2D 导航的复合系统原型

### 1.6 相关链接

| 资源 | 地址 |
|------|------|
| GitHub | https://github.com/quandy2020/autonomy |
| Gitee | https://gitee.com/quanduyong/autonomy |
| 在线文档 | https://autonomy.readthedocs.io |
| 许可证 | Apache 2.0 |
