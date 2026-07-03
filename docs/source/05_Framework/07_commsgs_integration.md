# 7. commsgs 集成

**commsgs** 是 Autonomy 的 Protobuf 消息层，贯穿配置、Server API 与 Autolink 传输。

### 7.1 角色

| 层次 | commsgs 用途 |
|------|----------------|
| 配置 | `proto::AutonomyOptions` 及各子模块 Options |
| Server API | `Path`、`PoseStamped`、`TwistStamped` 等 C++ 类型 |
| Autolink | Writer/Reader/Action 的序列化载荷 |
| ROS 2 Bridge | commsgs ↔ `rosidl` 转换 |

### 7.2 消息分层

```
commsgs/
├── core_msgs/       Pose, Header, Transform
├── geometry_msgs/   Point, Quaternion, Twist
├── sensor_msgs/     LaserScan, Image
├── map_msgs/        OccupancyGrid, Costmap
├── planning_msgs/   Path, Goal
└── nav_msgs/        Odometry
```

### 7.3 在 Framework 中的使用

```cpp
// Autonomy 导航 API
bool NavigateToPose(
    const commsgs::geometry_msgs::PoseStamped& goal, ...);

// PlannerServer
commsgs::planning_msgs::Path GetPlan(
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal, ...);
```

### 7.4 与 Autolink 的关系

- Autolink **不**定义业务消息，只负责通道与调度
- 话题类型为 `commsgs::xxx::Message` 的 Protobuf 描述
- Action 的 Goal/Result/Feedback 同样为 commsgs 类型

```
PlannerServer                    Autolink                     订阅方
    │                               │                            │
    ├── Writer<Path> ──────────────►│ topic: /plan ─────────────►│ RViz / Bridge
    └── ActionServer<GetPlan> ◄────►│ action: compute_path       │
```

### 7.5 配置中的 commsgs

Lua 加载后填充 Protobuf Options，例如：

```protobuf
message PlannerOptions {
  repeated string planner_plugins = 1;
  string default_planner = 2;
  map<string, PlannerPluginConfig> plugins = 3;
}
```

字段定义见 `autonomy/commsgs/proto/`。

### 7.6 ROS 2 互操作

通过 **Bridge** 模块将 commsgs 转为 ROS 2 消息，供 `rviz2`、`ros2 topic` 使用。Framework 本体不依赖 rclcpp。

详见 [14 Commsgs](../14_Commsgs/01_overview.md)、[15 Bridge](../15_Bridge/01_overview.md)。

### 7.7 相关文档

- [14 Commsgs 文档](../14_Commsgs/index.rst)
- [03 Communication](../03_Communication/01_architecture.md)
