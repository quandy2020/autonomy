# 4. 配置说明

可视化专用配置位于 `config/visualization/visualization.lua`。

> **注意**：该文件**尚未**被 `config/autonomy.lua` 引用，运行时 C++ 侧也**无**对应的 `LoadOptions` 解析。以下为**设计配置**，供未来 `VisualizationServer` 接入参考。

### 4.1 服务器参数

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `server_name` | `"AutonomyViewer"` | 服务名称 |
| `host` | `"0.0.0.0"` | 监听地址 |
| `port` | `8765` | WebSocket 端口 |

与 Docker `AUTONOMY_PORTS` 默认 `8765:8765` 对齐。

### 4.2 功能开关

| 字段 | 默认 | 说明 |
|------|------|------|
| `enable_client_publish` | `true` | 允许 Foxglove 反向发布 |
| `enable_connection_graph` | `true` | 连接图 |
| `enable_autolink` | `true` | 订阅 Autolink topic 并转发 |

### 4.3 topic_subscriptions

| topic | message_type | enabled |
|-------|--------------|---------|
| `/sensor/lidar` | LaserScan | ✅ |
| `/planning/path` | Path | ✅ |
| `/localization/odometry` | Odometry | ✅ |
| `/map` | OccupancyGrid | ✅ |
| `/sensor/camera` | Image | ✅ |
| `/sensor/pointcloud` | PointCloud2 | ❌ |
| `/sensor/imu` | Imu | ✅ |
| `/visualization/markers` | MarkerArray | ✅ |
| `/localization/pose` | PoseStamped | ✅ |

### 4.4 与栈内配置的对应

| visualization.lua | 其它配置 | 说明 |
|-------------------|----------|------|
| `/map` | `config/map/map.lua` → `map_topic` | 地图话题名可能为 `map` 而非 `/map` |
| `/planning/path` | 规划输出 | 进程内 API 为主，ROS 侧常为 `/plan` |
| `scan` | `config/planner/planner.lua` obstacle_layer | 激光话题 |

集成时需统一话题命名，参见 `config/common.lua`。

### 4.5 接入 autonomy.lua（规划）

```lua
include "visualization/visualization.lua"

AUTONOMY = {
  -- ...
  visualization = AUTONOMY_VISUALIZATION,
}
```

### 4.6 相关文档

- [§5 架构 · 目标数据流](05_architecture.md)
- [05 Framework · 配置管线](../05_Framework/04_configuration.md)
