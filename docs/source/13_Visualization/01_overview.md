(visualization-overview)=
# 1. 可视化概览

日常优先 [Autoviz](09_autoviz.md)（Autolink 直连的 Qt + Ogre 桌面程序，`BUILD_AUTOVIZ`）。也可以走 Foxglove（[06](06_foxglove.md)）或 RViz2（[07](07_rviz2_ros2.md)）。消息类型仍是 commsgs。

`autonomy/visualization` 里的 VisualizationServer 还没有。`visualization_msgs` 结构有，部分 ToProto 是 stub。Docker 里映射 8765 给 foxglove_bridge 的做法见运行文档。
