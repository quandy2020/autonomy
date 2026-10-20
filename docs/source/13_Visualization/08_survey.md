# 8. 可视化综述

见下文步骤中的现象与配置上下文。

**工具怎么选**

| 工具 | 适用 | 在本仓库怎么接 |
|------|------|----------------|
| **Autoviz** | 本机桌面、无 ROS | [09](09_autoviz.md)，`BUILD_AUTOVIZ` |
| **Foxglove** | 远程、多模态 | [06](06_foxglove.md)，常映射 8765 |
| **RViz2** | 已有 ROS 2 | [07](07_rviz2_ros2.md) + autonomy_ros |
| topic / glog | 排错、CI | 直接看日志或 echo |
| PlotJuggler | 控制时序 | 通常吃 ROS bag |

`autonomy/bridge` 是 **发令 / 状态** 的 gRPC，不是可视化通道；别和 foxglove_bridge 混用。

### 现状（务实版）

当前靠 **Autoviz / Foxglove / RViz2 + 日志** 就够联调。域内 VisualizationServer、Autolink 专用 Writer、完整 visualization_msgs 转换仍未产品化——有需求再跟代码，不必等「路线图阶段」。

相关链接：[Running · ROS 2](../04_Running/06_ros2_integration.md)、[Commsgs](../14_Commsgs/00_guide.md)、[Map](../07_Map/00_guide.md)。  
外部：[Foxglove](https://foxglove.dev/) · [RViz2](https://github.com/ros2/rviz) · [foxglove_bridge](https://github.com/foxglove/ros-foxglove-bridge)
