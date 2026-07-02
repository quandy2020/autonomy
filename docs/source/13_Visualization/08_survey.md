# 8. 可视化综述

### 8.1 工具选型

| 工具 | 优势 | 适用场景 | Autonomy 接入 |
|------|------|----------|---------------|
| **RViz2** | ROS 生态标准、TF/插件丰富 | 本地 ROS 2 开发 | autonomy_ros → rviz2 |
| **Foxglove** | 多模态、Web/远程、MCAP | 团队协作、远程调试 | foxglove_bridge |
| **ros2 topic echo** | 零配置 | 快速排错 | 直接 |
| **glog** | 无依赖 | 纯 C++ CI | autonomy_nav_test |
| **PlotJuggler** | 时序曲线强 | 控制调参 | ROS 2 bag / 实时 |

### 8.2 Bridge 与 Foxglove 对比

| 维度 | `autonomy/bridge` | `foxglove_bridge` |
|------|-------------------|-------------------|
| 协议 | gRPC / MQTT | WebSocket |
| 主要用途 | 远程导航 API | 可视化 |
| 仓库内实现 | ✅（gRPC stub 演进中） | ❌（ROS 包） |
| 与 Studio 直连 | ❌ | ✅ |

### 8.3 实现缺口与路线图

| 阶段 | 目标 |
|------|------|
| **当前** | ROS 2 + foxglove_bridge 可视化；纯 C++ 日志验证 |
| **近期** | 实现 `VisualizationServer` + 接入 `autonomy.lua` |
| **中期** | Autolink Writer 发布 map/path/odom；补全 `visualization_msgs` 转换 |
| **远期** | 内置 MCAP 录制；Foxglove 自定义面板 |

### 8.4 文档与资源索引

| 主题 | 章节 |
|------|------|
| 运行验证 | [04 Running · §7](../04_Running/07_verification.md) |
| ROS 2 集成 | [04 Running · §6](../04_Running/06_ros2_integration.md) |
| commsgs | [14 Commsgs](../14_Commsgs/index.rst) |
| Bridge | [15 Bridge](../15_Bridge/index.rst) |
| 地图 costmap | [07 Map](../07_Map/index.rst) |

### 8.5 外部链接

- [Foxglove 官网](https://foxglove.dev/)
- [RViz2 文档](https://github.com/ros2/rviz)
- [foxglove_bridge](https://github.com/foxglove/ros-foxglove-bridge)

### 8.6 源码与配置索引

| 路径 | 说明 |
|------|------|
| `config/visualization/visualization.lua` | 可视化配置（未接入） |
| `autonomy/commsgs/visualization_msgs.*` | Marker 消息定义 |
| `autonomy/map/costmap_2d/costmap_2d_wrapper.cpp` | RViz 兼容栅格 |
| `docker/install/install_ros2.sh` | foxglove-bridge 可选安装 |
| `docker/scripts/docker_utils.py` | 8765 端口映射 |
