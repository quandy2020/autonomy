# 5. 可视化架构

理解「业务进程为什么不带 GUI」以及数据如何到 Autoviz / ROS / Foxglove。

说明当前**实际出口**；域内 `VisualizationServer` **不存在**，`visualization.lua` **未加载**。

| 条件 | 工具 |
|------|------|
| 无 ROS | Autoviz |
| 远程浏览器 | Foxglove |
| 已有 ROS 工具链 | RViz2 |
| 只要日志 | glog + `ros2 topic echo` |

### 当前数据怎么被看见

| 来源 | 到显示端的路径 |
|------|----------------|
| Map / costmap | 内存快照、回调；无统一 Autolink「viz Writer」 |
| Planning | `GetPlan()` 等；ROS 侧由 autonomy_ros 发 `/plan` |
| Sensor → 导航 | 先进 ObstacleLayer，不是先进可视化 |
| 视觉 base/follow | Autolink 话题；Autoviz 或 bridge 订阅 |

栅格代价值与 RViz 兼容时，常见映射如 `INSCRIBED` → occupancy `99`（见 map 发布代码注释）。

### 三条推荐路径

```text
Stack ──Autolink──→ Autoviz（Qt+Ogre，无 ROS）

Stack ──autonomy_ros──→ ROS topics ──→ RViz2
                      └─→ foxglove_bridge ── ws://host:8765 ──→ Foxglove Studio
```

### 与 Bridge 的边界

| 模块 | 协议 | 用途 |
|------|------|------|
| `autonomy/bridge` | gRPC 等 | 远程发令、状态，**不是**画地图 |
| foxglove_bridge | WebSocket + ROS 2 | 可视化 |
| 规划中的 VisualizationServer | 未实现 | 勿等待 |

见 [15 Bridge](../15_Bridge/00_guide.md)。

### MCAP

`install_mcap.sh` 可装依赖；**业务 C++ 未**接 MCAP 录制 API。离线可看 ROS bag / 外部录包。

### 常见坑

| 现象 | 说明 |
|------|------|
| ps 找不到 VisualizationServer | 正常 |
| Foxglove 空 | bridge 未起或端口未映射 |
| 规划路径在 C++ 有、RViz 无 | autonomy_ros 是否 launch |

### 延伸阅读

[§6](06_foxglove.md) · [07 Map](../07_Map/00_guide.md)
