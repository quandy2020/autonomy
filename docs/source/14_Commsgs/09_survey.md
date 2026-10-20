(commsgs-survey)=
# 9. 消息体系综述

### 9.1 与 ROS 2 对照

| 维度 | ROS 2 | Autonomy |
|------|-------|----------|
| 定义 | .msg/.idl → rosidl | .proto + 手写 hpp |
| 序列化 / 中间件 | CDR / DDS | Protobuf / Autolink |
| 坐标系 | REP 103/105 | 继承 |

### 9.2 完整度矩阵

| 包 | struct/proto | ToProto | 综合 |
|----|--------------|---------|------|
| builtin / std | — | | ★★★★★ |
| geometry / planning | — | 大部分 / Path | ★★★★☆ |
| sensor / map / vision | — | | ★★★☆☆ |
| nav_msgs | SpeedLimit + proto Action | — | ★★★☆☆ |
| visualization / trajectory / diagnostic | — | stub | ★★☆☆☆ |
| shape / pcl / vehicle | 问题/空 | — | ★☆☆☆☆ |
| error_code | 仅 proto | N/A | ★★★★☆ |

已知 stub：`Polygon`/`PoseArray` 字段遗漏；sensor/map 不完整；visualization/trajectory 空函数体。

### 9.3 包映射（高频）

| ROS 2 | commsgs |
|-------|---------|
| geometry PoseStamped/Twist/TF | geometry_msgs 同名 |
| nav OccupancyGrid/Odometry/Path | map_msgs / planning_msgs |
| sensor LaserScan/Imu/PointCloud2 | sensor_msgs |
| nav2 NavigateToPose | `proto::nav_msgs::*Action` |

geometry 额外：`PointENU`/`PointLLH`/`Inertia`。

### 9.4 设计模式

Stamped、WithCovariance、child_frame_id、Action 三段式、Service、分段错误码、packed covariance、SmartPtr 宏。

### 9.5 选型与路线

| 场景 | 建议 |
|------|------|
| 栈内算法 | C++ struct |
| Autolink 服务 | proto + 边界转换 |
| 与 ROS 互通 | bridge，勿直连 commsgs |
| 性能路径 | 同进程传引用 |

| 阶段 | 目标 | 状态 |
|------|------|------|
| 1–2 | 定义 + 核心转换 | / |
| 3–4 | 传感器地图、viz/traj | — |
| 5–6 | bridge、shape 命名空间 | — |

### 9.6 参考

| 资源 | 链接 |
|------|------|
| common_interfaces / nav2_msgs / vision_msgs | GitHub ros2 / navigation2 / ros-perception |
| 通信 / 框架 | [03](../03_Communication/00_guide.md) · [05](../05_Framework/00_guide.md) |
