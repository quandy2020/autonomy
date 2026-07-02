(visualization-overview)=
# 1. 可视化概览

### 1.1 定位

Autonomy 核心库 `libautonomy` **不内置 GUI**，可视化通过外部工具消费 **commsgs** 数据（经桥接或直接发布）。

| 维度 | 说明 |
|------|------|
| 消息层 | [commsgs](../14_Commsgs/01_overview.md)（C++ struct + Protobuf） |
| 规划配置 | `config/visualization/visualization.lua`（**已编写，未接入运行时**） |
| 内置服务 | `VisualizationServer`（**规划中，源码未提交**） |
| 对标 | nav2_rviz_plugins、foxglove_bridge |

### 1.2 三条可视化路径

```
┌─────────────────────────────────────────────────────────────┐
│  路径 A：ROS 2 + RViz2（当前推荐）                            │
│  libautonomy → autonomy_ros（外部）→ rosidl → rviz2           │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  路径 B：Foxglove Studio（推荐多模态调试）                    │
│  ROS 2 topics → foxglove_bridge → WebSocket → Foxglove       │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  路径 C：纯 C++ 离线（无可视化 GUI）                          │
│  autonomy_nav_test → glog 日志验证                           │
└─────────────────────────────────────────────────────────────┘
```

### 1.3 实现状态对照

| 组件 | 状态 | 说明 |
|------|------|------|
| `autonomy/visualization/` 目录 | ❌ | 不存在 |
| `VisualizationServer` | ❌ | 仅文档示例 |
| `config/visualization/visualization.lua` | ⏳ | 存在，未 `include` 进 `autonomy.lua` |
| `visualization_msgs` struct/proto | ✅ | 定义完整 |
| `visualization_msgs` ToProto/FromProto | ❌ | cpp 空 stub |
| `Costmap2DPublisher` / 地图快照 | ⏳ | 生成 OccupancyGrid，无 Autolink Writer |
| `autonomy/bridge` | ✅ | gRPC/MQTT，**非** Foxglove 协议 |
| Docker 8765 端口 | ⏳ | 预留，与 visualization.lua 对齐 |
| `ros-foxglove-bridge` | ✅ | Docker 可选安装 |

### 1.4 可可视化数据（设计目标）

| 数据 | commsgs 类型 | 典型 topic |
|------|--------------|------------|
| 地图 | `map_msgs::OccupancyGrid` | `/map` |
| 路径 | `planning_msgs::Path` | `/planning/path` |
| 位姿/里程计 | `nav_msgs::Odometry` | `/localization/odometry` |
| 激光 | `sensor_msgs::LaserScan` | `/sensor/lidar` |
| 标记 | `visualization_msgs::MarkerArray` | `/visualization/markers` |
| 图像 | `sensor_msgs::Image` | `/sensor/camera` |

### 1.5 相关文档

- [§2 快速开始](02_quickstart.md)
- [04 Running · ROS 2](../04_Running/06_ros2_integration.md)
- [14 Commsgs · visualization_msgs](../14_Commsgs/08_nav_planning_msgs.md#85-visualization_msgs)
