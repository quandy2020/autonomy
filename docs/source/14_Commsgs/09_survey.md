(commsgs-survey)=
# 9. 消息体系综述

本文从 ROS 兼容性、实现完整度、设计取舍三个维度对 commsgs 消息体系进行综述。

## 9.1 设计定位

commsgs 在 Autonomy 栈中扮演 **ROS 2 common_msgs 的替代实现**：

| 维度 | ROS 2 | Autonomy commsgs |
|------|-------|------------------|
| 消息定义格式 | `.msg` / `.idl` | `.proto` + 手写 `.hpp` |
| 代码生成 | rosidl 统一生成 | protoc 生成 proto + 手写 struct |
| 运行时类型 | 单一 ROS 消息类型 | C++ struct（算法）+ proto（传输） |
| 序列化 | CDR (DDS) | Protocol Buffers |
| 通信中间件 | DDS (Fast-DDS 等) | Autolink |
| 坐标系约定 | REP 103/105 | 完全继承 |
| 依赖 | rclcpp + rosidl | 仅 protobuf + 标准库 |

**核心取舍**：牺牲 ROS 生态即插即用，换取更轻量的依赖链与显式的序列化边界。

## 9.2 实现完整度矩阵

### 9.2.1 消息包级别

| 包 | struct 定义 | proto 定义 | ToProto/FromProto | 综合 |
|----|------------|-----------|-------------------|------|
| `builtin_interfaces` | ✅ | ✅ | ✅ 完整 | ★★★★★ |
| `std_msgs` | ✅ | ✅ | ✅ 完整 | ★★★★★ |
| `geometry_msgs` | ✅ | ✅ | ✅ 大部分完整 | ★★★★☆ |
| `planning_msgs` | ✅ | ✅ | ✅ Path 完整 | ★★★★☆ |
| `sensor_msgs` | ✅ | ✅ | ⏳ 部分 stub | ★★★☆☆ |
| `map_msgs` | ✅ | ✅ | ⏳ 部分 stub | ★★★☆☆ |
| `nav_msgs` | ⏳ 仅 SpeedLimit | ✅ Action/Service 完整 | ⏳ | ★★★☆☆ |
| `vision_msgs` | ✅ | ✅ | ⏳ 部分 | ★★★☆☆ |
| `visualization_msgs` | ✅ | ✅ | ❌ cpp 空 stub | ★★☆☆☆ |
| `trajectory_msgs` | ✅ | ✅ | ❌ cpp 空 stub | ★★☆☆☆ |
| `diagnostic_msgs` | ✅ | ✅ | ❌ cpp 空 stub | ★★☆☆☆ |
| `stereo_msgs` | ✅ | ✅ | ⏳ 低 | ★★☆☆☆ |
| `shape_msgs` | ⏳ 命名空间错误 | ✅ | ❌ cpp 空 stub | ★☆☆☆☆ |
| `pcl_msgs` | ❌ 空 | ✅ | ❌ 空 | ★☆☆☆☆ |
| `vehicle_msgs` | ⏳ 空 struct | ✅ | ⏳ 低 | ★☆☆☆☆ |
| `error_code` | ❌ 仅 proto | ✅ | N/A | ★★★★☆ (proto) |

### 9.2.2 已知 stub 示例

| 位置 | 问题 |
|------|------|
| `geometry_msgs.cpp` | `Polygon ToProto` 未填充 points |
| `geometry_msgs.cpp` | `PoseArray ToProto` 未设置 header |
| `sensor_msgs.cpp` | 多个传感器消息转换不完整 |
| `map_msgs.cpp` | `OccupancyGrid`/`Costmap` 部分字段遗漏 |
| `trajectory_msgs.cpp` | 全部为空函数体 |
| `visualization_msgs.cpp` | 全部为空函数体 |

## 9.3 ROS 2 消息对照表

### 9.3.1 包映射

| ROS 2 包 | commsgs 包 | 差异 |
|----------|-----------|------|
| `builtin_interfaces` | `builtin_interfaces` | 一致 |
| `std_msgs` | `std_msgs` | 一致 |
| `geometry_msgs` | `geometry_msgs` | 新增 `PointENU`/`PointLLH`/`Inertia` |
| `sensor_msgs` | `sensor_msgs` | 一致；`BatteryState` 为 proto 别名 |
| `nav_msgs` | `map_msgs` + `planning_msgs` | OccupancyGrid/Odometry 拆分 |
| `nav2_msgs` | `proto::nav_msgs` | Action/Service 在 proto 层 |
| `vision_msgs` | `vision_msgs` | 一致 |
| `visualization_msgs` | `visualization_msgs` | 一致 |
| `trajectory_msgs` | `trajectory_msgs` | 一致 |
| `diagnostic_msgs` | `diagnostic_msgs` | 一致 |
| `stereo_msgs` | `stereo_msgs` | 一致 |
| `shape_msgs` | `shape_msgs` | 命名空间 bug |
| `pcl_msgs` | `pcl_msgs` | 未实现 |

### 9.3.2 高频消息对照

| ROS 2 消息 | commsgs 对应 | 字段兼容 |
|-----------|-------------|----------|
| `geometry_msgs/PoseStamped` | `geometry_msgs::PoseStamped` | ✅ |
| `geometry_msgs/Twist` | `geometry_msgs::Twist` | ✅ |
| `geometry_msgs/TransformStamped` | `geometry_msgs::TransformStamped` | ✅ |
| `nav_msgs/OccupancyGrid` | `map_msgs::OccupancyGrid` | ✅ |
| `nav_msgs/Odometry` | `planning_msgs::Odometry` | ✅ |
| `nav_msgs/Path` | `planning_msgs::Path` | ✅ |
| `sensor_msgs/LaserScan` | `sensor_msgs::LaserScan` | ✅ |
| `sensor_msgs/Imu` | `sensor_msgs::Imu` | ✅ |
| `sensor_msgs/PointCloud2` | `sensor_msgs::PointCloud2` | ✅ |
| `nav2_msgs/action/NavigateToPose` | `proto::nav_msgs::NavigateToPoseAction` | ✅ |

## 9.4 消息设计模式总结

| 模式 | 示例 | 用途 |
|------|------|------|
| Stamped | `PoseStamped`, `TwistStamped` | 时间 + 坐标系 |
| WithCovariance | `PoseWithCovariance` | 不确定性估计 |
| child_frame_id | `Odometry`, `TransformStamped` | 父子坐标系 |
| Action 三段式 | `Goal` / `Feedback` / `Result` | 长时任务 |
| Service 请求/响应 | `IsPathValid` | 同步查询 |
| 分段错误码 | `error_code.ErrorCode` | 跨模块统一错误语义 |
| packed repeated | `repeated float covariance` | 协方差高效序列化 |
| SmartPtr 宏 | `AUTONOMY_SMART_PTR_DEFINITIONS` | ROS 2 风格 API |

## 9.5 与其他中间件对比

| 方案 | 优势 | 劣势 |
|------|------|------|
| **commsgs (当前)** | 轻量、无 DDS 依赖、显式边界 | 不兼容 ROS 生态 |
| ROS 2 rosidl | 生态成熟、工具链完整 | 依赖重、CDR 性能一般 |
| FlatBuffers | 零拷贝、高性能 | 生态较小 |
| Cap'n Proto | 零拷贝、RPC 内建 | C++ 支持好但机器人生态弱 |
| Protobuf only | 简单统一 | 算法层有 protobuf 运行时开销 |

commsgs 的 **双层设计** 兼顾了 protobuf 序列化优势与 C++ 算法层的零开销需求。

## 9.6 演进路线

| 阶段 | 目标 | 状态 |
|------|------|------|
| Phase 1 | 核心消息 struct + proto 定义 | ✅ 完成 |
| Phase 2 | 核心包 ToProto/FromProto 完整 | ⏳ 进行中 |
| Phase 3 | 传感器/地图转换补全 | ⏳ 待做 |
| Phase 4 | visualization/trajectory 转换 | ⏳ 待做 |
| Phase 5 | bridge 模块 ROS ↔ commsgs | ⏳ 依赖 bridge |
| Phase 6 | shape_msgs 命名空间修复 | ⏳ 待做 |

## 9.7 选型建议

| 场景 | 建议 |
|------|------|
| 导航栈内部开发 | 直接使用 C++ struct，无需关心 proto |
| Autolink 服务开发 | 使用 proto 类型，边界转换 |
| 新增消息 | 遵循 proto → hpp → cpp 三文件模式 |
| 与 ROS 系统互通 | 通过 `bridge` 模块转换，不直接使用 commsgs |
| 单元测试 | 对关键消息编写 ToProto/FromProto 往返测试 |
| 性能敏感路径 | 进程内传递 struct 引用，避免 proto 转换 |

## 9.8 参考资源

| 资源 | 链接 |
|------|------|
| ROS 2 common_msgs | [ros2/common_interfaces](https://github.com/ros2/common_interfaces) |
| Nav2 msgs | [ros-navigation/navigation2/nav2_msgs](https://github.com/ros-navigation/navigation2/tree/main/nav2_msgs) |
| vision_msgs | [ros-perception/vision_msgs](https://github.com/ros-perception/vision_msgs) |
| Autonomy 通信框架 | [03 Communication](../03_Communication/00_guide.md) |
| Autonomy 框架集成 | [05 Framework 文档](../05_Framework/index.rst) |
| 源码 | [`autonomy/commsgs/`](https://github.com/quandy2020/autonomy/tree/feature/library/autonomy/commsgs) |
