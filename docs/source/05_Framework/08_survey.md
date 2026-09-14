# 8. 综述与演进

### 8.1 框架能力矩阵

| 能力 | 状态 | 说明 |
|------|------|------|
| 多进程 launch | ✅ 可用 | `autolink_launch autonomy.launch` |
| `CreateOptions` / `AutonomyOptions` | ✅ 可用 | 共享 conf 快照 |
| MapServer | ✅ 可用 | 静态地图 + costmap 回调 |
| PlannerServer + 插件 | ✅ 可用 | navfn、theta_star 等 |
| ControllerServer + 插件 | ✅ 可用 | regulated_pure_pursuit 等 |
| TransformServer | ✅ 可用 | 静态 TF |
| TaskServer + BT | ✅/⏳ | task 进程可用，BT 能力持续演进 |
| 进程内 `CreateAutonomy` | ❌ 已移除 | 不再聚合整栈 |

### 8.2 与 ROS 2 Nav2 对比

| 维度 | Autonomy Framework | Nav2 |
|------|-------------------|------|
| 配置 | Protobuf / 文本 conf | YAML + 参数服务器 |
| 通信 | Autolink + commsgs | ROS 2 DDS |
| 编排 | TaskServer + BT | BehaviorTree.CPP |
| 插件 | PluginManager + .so | pluginlib |
| 入口 | `autolink_launch` + 各 `*_main` | `nav2_bringup` lifecycle nodes |

### 8.3 已知限制

1. **发令需外部客户端**：主栈不自动下发目标；用 Bridge / Action
2. **多进程为主**：不再提供单进程 `autonomy_nav_test` / `CreateAutonomy`
3. **Autolink 与 Framework 文档分离**：通信 API 见 [03 Communication](../03_Communication/index.rst)

### 8.4 演进路线

| 阶段 | 目标 |
|------|------|
| 近期 | Bridge / Action 端到端发令文档与示例 |
| 中期 | 各 Server 生命周期与热重载 |
| 远期 | 多机器人命名空间 |

### 8.5 文档索引

| 主题 | 章节 |
|------|------|
| 系统总览 | [01 Instructions](../01_Instructions/01_overview.md) |
| 运行与调试 | [04 Running](../04_Running/index.rst) |
| 通信层 | [03 Communication](../03_Communication/index.rst) |
| 各模块 Server | [07 Map](../07_Map/index.rst) · [08 Planning](../08_Planning/index.rst) · [09 Control](../09_Control/index.rst) |
| Task / Navigator | [17 Tasks](../17_Tasks/index.rst) · [16 Navigator](../16_Navigator/index.rst) |
| 消息定义 | [14 Commsgs](../14_Commsgs/index.rst) |

### 8.6 源码入口

| 路径 | 说明 |
|------|------|
| `autonomy/system/options.*` | `CreateOptions` / 共享 conf |
| `autonomy/system/launch/autonomy.launch` | 多进程入口 |
| `autonomy/system/conf/autonomy.pb.txt` | 共享快照 |
| `autonomy/planning/planning_main.cpp` | 规划进程 |
| `autonomy/control/control_main.cpp` | 控制进程 |
| `autonomy/task/task_main.cpp` | 任务进程 |
