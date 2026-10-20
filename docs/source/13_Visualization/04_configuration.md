# 4. 配置说明

文件：`config/visualization/visualization.lua`。**未**被 `autonomy.lua` 引用，无 `LoadOptions`。

### 4.1 服务器与开关

| 字段 | 默认 | 说明 |
|------|------|------|
| `server_name` | `"AutonomyViewer"` | 服务名 |
| `host` / `port` | `0.0.0.0` / `8765` | WebSocket（对齐 Docker） |
| `enable_client_publish` | true | 反向发布 |
| `enable_connection_graph` | true | 连接图 |
| `enable_autolink` | true | 订阅 Autolink 转发 |

### 4.2 topic_subscriptions

| topic | message_type | enabled |
|-------|--------------|---------|
| `/sensor/lidar` | LaserScan | — |
| `/planning/path` | Path | — |
| `/localization/odometry` | Odometry | — |
| `/map` | OccupancyGrid | — |
| `/sensor/camera` | Image | — |
| `/sensor/pointcloud` | PointCloud2 | — |
| `/sensor/imu` | Imu | — |
| `/visualization/markers` | MarkerArray | — |
| `/localization/pose` | PoseStamped | — |

注意：地图可能为 `map` 而非 `/map`；ROS 路径常为 `/plan`。统一见 `config/common.lua`。

### 4.3 接入（规划）

```lua
include "visualization/visualization.lua"
AUTONOMY = { visualization = AUTONOMY_VISUALIZATION }
```

### 4.4 参见

- [§5](05_architecture.md) · [05 Framework · 配置](../05_Framework/04_configuration.md)
