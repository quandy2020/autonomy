# 5. 可视化架构

### 5.1 目标（规划）

```
Map/Planner/Controller/Sensor → VisualizationServer (ws:8765) → Foxglove
Stack → autonomy_ros → RViz2
```

职责：订 Autolink / 回调 → 序列化 → WebSocket；可选 MCAP（依赖已装，C++ 未接）。

### 5.2 当前

| 出口 | 方式 |
|------|------|
| Map | 回调 → costmap；无 Writer |
| Planning | `GetPlan()` 返回值 |
| Costmap | `snapshotOccupancyGrid()` 内存快照 |
| Sensor | → ObstacleLayer |

无统一 Autolink Writer。栅格：`INSCRIBED`→ occupancy `99`（RViz 兼容）。`Costmap2DPublisher` ROS 发布已注释。

### 5.3 Bridge 边界

| 模块 | 协议 | 用途 |
|------|------|------|
| `autonomy/bridge` | gRPC/MQTT | 远程导航 |
| `foxglove_bridge` | WebSocket+ROS2 | 可视化 |
| `VisualizationServer`（规划） | WebSocket+commsgs | 原生可视化 |

互不替代。见 [15 Bridge](../15_Bridge/00_guide.md)。

### 5.4 参见

- [§6](06_foxglove.md) · [07 Map](../07_Map/00_guide.md) · [08 Planning](../08_Planning/00_guide.md)
