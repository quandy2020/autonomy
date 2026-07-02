(commsgs-overview)=
# 1. 模块概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 层级 | 基础设施层 — 全栈共享的消息类型定义 |
| 输入 | 无（纯数据模型，不依赖运行时） |
| 输出 | C++ struct、`.proto` schema、转换函数 `ToProto` / `FromProto` |
| 上游 | 无 |
| 下游 | `planning`、`map`、`control`、`localization`、`transform`、`visualization`、`bridge` |
| 对标 | ROS 2 `common_msgs`、`nav2_msgs`、`vision_msgs` |

### 1.2 核心能力

| 能力 | 状态 | 说明 |
|------|------|------|
| 16 个消息包（namespace） | ✅ 已定义 | 对齐 ROS 命名习惯 |
| C++ struct 手写定义 | ✅ 已定义 | 算法层直接使用，无 protobuf 运行时依赖 |
| Proto schema | ✅ 已定义 | Autolink / 网络序列化 |
| `ToProto` / `FromProto` 转换 | ⏳ 部分 | 核心几何/标准消息较完整，传感器/地图等待补全 |
| `PointCloud2` 迭代器 | ✅ 已实现 | 从 ROS `sensor_msgs` 移植 |
| Nav2 Action/Service schema | ✅ proto 层 | `nav_msgs.proto` 定义 Goal/Feedback/Result |
| 统一错误码 | ✅ proto 层 | `error_code.proto`，Apollo 风格分段 |
| ROS 序列化兼容 | ❌ 不支持 | 不使用 `rosidl`，走 protobuf |

> **当前阶段**：消息 struct 与 proto 定义已覆盖导航栈主要场景；部分包的 `ToProto`/`FromProto` 仍为 stub，详见 [§9 综述](09_survey.md#92-实现完整度矩阵)。

### 1.3 源码结构

```
autonomy/commsgs/
├── proto/                          # 16 个 .proto（wire / 序列化层）
│   ├── builtin_interfaces.proto
│   ├── std_msgs.proto
│   ├── geometry_msgs.proto
│   ├── sensor_msgs.proto
│   ├── map_msgs.proto
│   ├── nav_msgs.proto
│   ├── planning_msgs.proto
│   ├── trajectory_msgs.proto
│   ├── diagnostic_msgs.proto
│   ├── visualization_msgs.proto
│   ├── vision_msgs.proto
│   ├── shape_msgs.proto
│   ├── stereo_msgs.proto
│   ├── pcl_msgs.proto
│   ├── vehicle_msgs.proto
│   └── error_code.proto
├── impl/
│   └── point_cloud2_iterator.hpp   # PointCloud2 迭代器模板实现
├── builtin_interfaces.{hpp,cpp}
├── std_msgs.{hpp,cpp}
├── geometry_msgs.{hpp,cpp}         # 最完整的消息包
├── sensor_msgs.{hpp,cpp}
├── map_msgs.{hpp,cpp}
├── planning_msgs.{hpp,cpp}
├── nav_msgs.{hpp,cpp}
├── vision_msgs.{hpp,cpp}
├── visualization_msgs.{hpp,cpp}
├── trajectory_msgs.{hpp,cpp}
├── diagnostic_msgs.{hpp,cpp}
├── shape_msgs.{hpp,cpp}
├── stereo_msgs.{hpp,cpp}
├── pcl_msgs.{hpp,cpp}
├── vehicle_msgs.{hpp,cpp}
├── point_cloud2_iterator.hpp       # PointCloud2 工具（ROS 移植）
└── point_field_conversion.hpp      # PointField 类型映射
```

### 1.4 导航栈数据流

```
传感器驱动 ──→ sensor_msgs ──→ map / perception
                                    ↓
静态地图 ──→ map_msgs::OccupancyGrid ──→ Costmap2D
                                    ↓
规划器 ──→ planning_msgs::Path ──→ Controller
                                    ↓
TF ──→ geometry_msgs::TransformStamped ──→ 全栈坐标变换
                                    ↓
Autolink 边界 ──→ proto ──→ 网络 / 跨进程
```

### 1.5 消息包一览

| 包名 | C++ 命名空间 | 典型用途 | 完整度 |
|------|-------------|----------|--------|
| `builtin_interfaces` | `commsgs::builtin_interfaces` | 时间、时长 | ✅ 高 |
| `std_msgs` | `commsgs::std_msgs` | Header、ColorRGBA | ✅ 高 |
| `geometry_msgs` | `commsgs::geometry_msgs` | 位姿、速度、变换 | ✅ 高 |
| `sensor_msgs` | `commsgs::sensor_msgs` | 激光、IMU、点云、图像 | ⏳ 中 |
| `map_msgs` | `commsgs::map_msgs` | 栅格地图、代价地图、GridMap | ⏳ 中 |
| `planning_msgs` | `commsgs::planning_msgs` | Path、Odometry、Goals | ✅ 中 |
| `nav_msgs` | `commsgs::nav_msgs` | SpeedLimit；Action 在 proto | ⏳ proto 为主 |
| `vision_msgs` | `commsgs::vision_msgs` | 2D/3D 检测框 | ⏳ 中 |
| `visualization_msgs` | `commsgs::visualization_msgs` | Marker、RViz 可视化 | ⏳ stub |
| `trajectory_msgs` | `commsgs::trajectory_msgs` | 关节轨迹 | ⏳ stub |
| `diagnostic_msgs` | `commsgs::diagnostic_msgs` | 诊断信息 | ⏳ stub |
| `stereo_msgs` | `commsgs::stereo_msgs` | 视差图 | ⏳ 低 |
| `shape_msgs` | `commsgs::shape_msgs` | 几何体形状 | ⏳ 低 |
| `pcl_msgs` | `commsgs::pcl_msgs` | PCL 相关 | ⏳ 空 |
| `vehicle_msgs` | `commsgs::vehicle_msgs` | 车辆状态 | ⏳ 空 |
| `error_code` | `commsgs::proto::error_code` | 统一错误码（仅 proto） | ✅ proto |

### 1.6 相关模块

| 模块 | 关系 |
|------|------|
| `autonomy/planning` | 使用 `planning_msgs::Path`、`proto::nav_msgs` Action/Service |
| `autonomy/map` | 使用 `map_msgs`、`sensor_msgs::PointCloud2` |
| `autonomy/control` | 使用 `geometry_msgs`、`planning_msgs` |
| `autonomy/transform` | 使用 `geometry_msgs::TransformStamped` |
| `autonomy/visualization` | 发布多种 commsgs 到可视化服务 |
| `autonomy/bridge` | ROS ↔ Autonomy 消息桥接 |
| Autolink | 进程间通信，wire 格式为 protobuf |

### 1.7 与 ROS 2 对照

| ROS 2 包 | Autonomy 对应 | 备注 |
|----------|---------------|------|
| `builtin_interfaces` | `commsgs::builtin_interfaces` | `Time`/`Duration` 语义一致 |
| `std_msgs` | `commsgs::std_msgs` | `Header` 为核心 |
| `geometry_msgs` | `commsgs::geometry_msgs` | 含 Apollo 扩展 `PointENU`/`PointLLH` |
| `sensor_msgs` | `commsgs::sensor_msgs` | 含 `PointCloud2` 工具 |
| `nav_msgs` | `commsgs::map_msgs` + `planning_msgs` | OccupancyGrid 在 map_msgs |
| `nav2_msgs` | `commsgs::proto::nav_msgs` | Action/Service 在 proto 层 |
| `vision_msgs` | `commsgs::vision_msgs` | 检测/分类消息 |
| `visualization_msgs` | `commsgs::visualization_msgs` | Marker 系列 |
