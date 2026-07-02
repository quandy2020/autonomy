# 8. 综述与演进

### 8.1 框架能力矩阵

| 能力 | 状态 | 说明 |
|------|------|------|
| `system::Autonomy` 生命周期 | ✅ 可用 | Start / Configure / Shutdown |
| Lua → Protobuf 配置 | ✅ 可用 | `autonomy.lua` 聚合 |
| MapServer | ✅ 可用 | 静态地图 + costmap 回调 |
| PlannerServer + 插件 | ✅ 可用 | navfn、theta_star 等 |
| ControllerServer + 插件 | ✅ 可用 | regulated_pure_pursuit 等 |
| TransformServer | ✅ 可用 | 静态 TF |
| 直驱导航 API | ✅ 可用 | GetPlan + NotifyPath |
| BT 导航 | ⚠️ 演进中 | 栈待恢复，默认 `use_bt_navigation = false` |
| SensorCollator | ⚠️ 演进中 | 激光聚合接口存在 |
| 多机器人 | ❌ 未支持 | 单 `Autonomy` 实例 |

### 8.2 与 ROS 2 Nav2 对比

| 维度 | Autonomy Framework | Nav2 |
|------|-------------------|------|
| 配置 | Lua → Protobuf | YAML + 参数服务器 |
| 通信 | Autolink + commsgs | ROS 2 DDS |
| 编排 | navigator BT（演进） | BehaviorTree.CPP |
| 插件 | PluginManager + .so | pluginlib |
| 入口 | `system::Autonomy` | `nav2_bringup` lifecycle nodes |

### 8.3 已知限制

1. **BT 导航未默认启用**：需 `Configure({use_bt_navigation=true})` 且 BT 栈完整
2. **单进程为主**：`autonomy_nav_test` 单进程集成各 Server
3. **Autolink 与 Framework 文档分离**：通信 API 见 [03 Communication](../03_Communication/index.rst)

### 8.4 演进路线

| 阶段 | 目标 |
|------|------|
| 近期 | 恢复 BT 节点与 `NavigateToPose` 端到端 |
| 中期 | Server 可选进程外部署（纯 Autolink 模式） |
| 远期 | 多机器人命名空间、生命周期与热重载 |

### 8.5 文档索引

| 主题 | 章节 |
|------|------|
| 系统总览 | [01 Instructions](../01_Instructions/01_overview.md) |
| 运行与调试 | [04 Running](../04_Running/index.rst) |
| 通信层 | [03 Communication](../03_Communication/index.rst) |
| 各模块 Server | [07 Map](../07_Map/index.rst) · [08 Planning](../08_Planning/index.rst) · [09 Control](../09_Control/index.rst) |
| Navigator | [16 Navigator](../16_Navigator/index.rst) |
| 消息定义 | [14 Commsgs](../14_Commsgs/index.rst) |

### 8.6 源码入口

| 路径 | 说明 |
|------|------|
| `autonomy/system/autonomy.{hpp,cpp}` | 主类 |
| `autonomy/system/options.cpp` | 配置加载 |
| `config/autonomy.lua` | 顶层配置 |
| `autonomy/planning/planner_server.cpp` | 规划 Server + 插件 |
| `autonomy/control/controller_server.cpp` | 控制 Server |
