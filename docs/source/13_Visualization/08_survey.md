# 8. 可视化综述

### 8.1 工具选型

| 工具 | 适用 | Autonomy |
|------|------|----------|
| RViz2 | 本地 ROS2 | autonomy_ros → rviz2 |
| Foxglove | 远程、多模态、MCAP | foxglove_bridge |
| topic echo / glog | 排错 / CI | 直接 |
| PlotJuggler | 控制时序 | ROS bag |

| 维度 | `autonomy/bridge` | `foxglove_bridge` |
|------|-------------------|-------------------|
| 协议 | gRPC/MQTT | WebSocket |
| 用途 | 远程导航 | 可视化 |
| 仓库内 | — | （ROS 包） |

### 8.2 路线图

| 阶段 | 目标 |
|------|------|
| 当前 | ROS2 + foxglove_bridge；C++ 日志 |
| 近 | VisualizationServer + autonomy.lua |
| 中 | Autolink Writer；visualization_msgs 转换 |
| 远 | MCAP 录制；自定义面板 |

### 8.3 索引

| 主题 | 文档 / 路径 |
|------|-------------|
| 运行 / ROS2 | [04 §6–§7](../04_Running/06_ros2_integration.md) |
| commsgs / Bridge / Map | [14](../14_Commsgs/00_guide.md) · [15](../15_Bridge/00_guide.md) · [07](../07_Map/00_guide.md) |
| 配置 / 消息 | `config/visualization/visualization.lua`、`commsgs/visualization_msgs.*` |
| 栅格兼容 / 端口 | `costmap_2d_wrapper.cpp`、`docker_utils.py` 8765 |

外部：[Foxglove](https://foxglove.dev/) · [RViz2](https://github.com/ros2/rviz) · [foxglove_bridge](https://github.com/foxglove/ros-foxglove-bridge)
