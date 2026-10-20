(visualization-overview)=
# 1. 可视化概览

选 Autoviz、Foxglove 还是 RViz2，并确认没有「域内可视化 Server」。

业务进程（planning/control/map…）**不带 GUI**。要看地图、路径、激光，必须额外起显示端。

| 路径 | 适合 | 入口 |
|------|------|------|
| Autoviz | 本机桌面、无 ROS | [§9](09_autoviz.md) |
| Foxglove | 浏览器、远程 | [§6](06_foxglove.md) |
| RViz2 | 已有 ROS 2 | [§7](07_rviz2_ros2.md) |

消息语义来自 commsgs（[14](../14_Commsgs/00_guide.md)）。`VisualizationServer` **未实现**；`visualization.lua` **未**被主配置加载。

无 ROS → Autoviz；要网页 → Foxglove + bridge；已是 ROS 工具链 → RViz2。

### 常见坑

| 误解 | 事实 |
|------|------|
| ps 应有 VisualizationServer | 否 |
| 改 visualization.lua 就能订话题 | lua 未接线 |

### 延伸阅读

[00 指南](00_guide.md)
