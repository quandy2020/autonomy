# 4. 术语表

| 术语 | 说明 |
|------|------|
| **Autonomy** | 本项目的机器人导航框架（`libautonomy`） |
| **Autolink** | 通信中间件，提供 Node / Channel / Action |
| **commsgs** | Protobuf 消息层，兼容 ROS 消息语义 |
| **Server** | 子系统服务入口（MapServer、PlannerServer 等） |
| **Navigator** | 行为树**导航编排**模块 |
| **BT** | Behavior Tree，行为树 |
| **Task** | 导航任务（NavigateToPose 等），非 `common::Task` |
| **Costmap** | 代价地图，用于规划与避障 |
| **TF** | 坐标变换（Transform） |
| **Nav2** | ROS 2 Navigation2 栈，Autonomy 的主要对标 |
| **Plugin** | 动态加载的算法实现（规划器、控制器等） |
| **Lua → Protobuf** | 配置加载管线：Lua 脚本 → Options Proto |
| **直驱模式** | 不经 BT，Autonomy 直接调用 Planner |
| **Bridge** | 外部系统桥接（gRPC 等） |

### 4.1 帧名约定

| 帧名 | 典型值 | 说明 |
|------|--------|------|
| `global_frame` | `map` | 全局地图坐标系 |
| `robot_base_frame` | `base_link` | 机器人基座 |
| `odom_frame` | `odom` | 里程计坐标系 |

定义于 `config/common.lua`，各模块须保持一致。
