(commsgs-survey)=
# 9. 消息体系综述

联调前要确认「这个包能不能过 Autolink 边界」、该用 struct 还是 proto Action。

对照 ROS 2 常见包，说明 commsgs 里 **struct/proto 是否齐全、ToProto 是否可信**。

| 场景 | 建议 |
|------|------|
| 同进程算法 | C++ struct，零拷贝引用 |
| Autolink Pub/Sub | struct（框架内序列化） |
| Service / Action | 边界 `FromProto` / `ToProto` |
| 与 ROS 互通 | autonomy_ros bridge，勿假设 commsgs 与 rosidl 比特级一致 |

### 与 ROS 2 差异

| 维度 | ROS 2 | Autonomy commsgs |
|------|-------|------------------|
| 定义 | .msg / rosidl | `.proto` + 手写 hpp |
| 线上格式 | CDR | Protobuf |
| 坐标约定 | REP 103/105 | 语义继承 |

### 包选型（按联调把握）

| 包 | struct + proto | ToProto/FromProto | 建议 |
|----|----------------|-------------------|------|
| builtin_interfaces / std_msgs | 完整 | 完整 | 默认优先 |
| geometry_msgs / planning_msgs | 完整 | 大部分完整 | 路径、位姿、TF |
| sensor_msgs / map_msgs | 完整 | 部分字段/类型需实测 | 激光、栅格上线前往返测 |
| vision_msgs | 有 | 部分 | 视觉联调前测 |
| nav_msgs | SpeedLimit + proto Action | Action 走 proto；SpeedLimit 常用 | 导航 Action 用 `proto::nav_msgs::*` |
| visualization_msgs / trajectory_msgs / diagnostic_msgs | 有定义 | 多为 stub 空体 | 勿默认能过边界 |
| shape_msgs | 有 | 有已知命名空间问题 | 慎用 |
| pcl_msgs / vehicle_msgs | 占位 | 无 | 勿依赖 |
| error_code | 仅 proto | N/A | Bridge / Action 结果 |

「stub」指 `ToProto`/`FromProto` 空实现或未填字段；表现常为**编译过、对端全零**。

### 已知问题（联调前扫一眼）

- `Polygon` / `PoseArray` 等部分 geometry 转换字段遗漏。  
- sensor / map 大包体消息需测性能与字段对齐。  
- visualization / trajectory：`Marker` 等 **stub**。  
- shape_msgs：hpp 命名空间与 proto 不一致的历史问题。

细节字段见 [§3 Schema](03_schema.md)、分册 [§6–§8](06_core_msgs.md)。

### ROS 2 → commsgs 高频映射

| ROS 2 | commsgs |
|-------|---------|
| geometry PoseStamped / Twist / TF | geometry_msgs 同名 |
| nav OccupancyGrid / Odometry / Path | map_msgs / planning_msgs |
| sensor LaserScan / Imu / PointCloud2 | sensor_msgs |
| nav2 NavigateToPose | `commsgs::proto::nav_msgs::*Action` |

geometry 额外：`PointENU`、`PointLLH`、`Inertia`（Apollo 风格扩展）。

### 常见坑

| 现象 | 先查 |
|------|------|
| 对端解包全 0 | 是否 stub ToProto |
| ROS 字段少一截 | bridge 转换表，不是 commsgs 同名即同布局 |
| 找 gRPC 探索字段 | automsgs.rpcs，见 [15 Bridge](../15_Bridge/00_guide.md) |

### 延伸阅读

[§3 Schema](03_schema.md) · [03 Communication](../03_Communication/00_guide.md)
