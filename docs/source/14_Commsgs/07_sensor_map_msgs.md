# 7. 传感器与地图消息

接激光、点云、OccupancyGrid 或 Follow 的 GridMap 时对字段。

sensor_msgs / map_msgs 支撑 **costmap**、定位与 **perception/base** 输入；ToProto 对部分类型需联调验证。

| 场景 | 消息 |
|------|------|
| 2D 导航地图 | OccupancyGrid 或内部 Costmap |
| 激光避障 | LaserScan |
| 3D 障碍 | PointCloud2 + VoxelLayer |
| 跟人 MPPI | GridMap（follow 话题） |

### sensor_msgs 要点

| 类型 | 关键字段 |
|------|----------|
| `LaserScan` | `angle_*`, `range_*`, `ranges` |
| `Imu` | orientation, angular_velocity, linear_acceleration；协方差 -1=未知 |
| `PointCloud2` | `fields`, `point_step`, `row_step`, `data` |
| `Image` / `CompressedImage` | encoding, step, data |
| `CameraInfo` | K, D, R, P |

旧 `PointCloud` 已 deprecated。点云工具：`PointCloud2Modifier`、`PointCloud2Iterator`（`obstacle_layer`、`voxel_layer` 使用）。

### map_msgs 要点

| 类型 | 说明 |
|------|------|
| `OccupancyGrid` | `info` + `data`：-1 未知，0 自由，100 占据 |
| `OccupancyGridUpdate` | 矩形增量 |
| `Costmap` / Update | Nav2 风格代价值 0–255 |
| `GridMap` | 多层 float（如 elevation、obstacle） |
| `Octomap` 等 | 3D / 辅助 |

**转换：** OccupancyGrid / Costmap / GridMap 常用；大包体注意性能。

### 数据流

```text
LaserScan / PointCloud2 → obstacle_layer / voxel_layer → Costmap2D
OccupancyGrid ← 静态地图 / 发布快照
GridMap ← perception/follow（不进 OccupancyGrid）
```

### 常见坑

| 现象 | 说明 |
|------|------|
| PointCloud 越界 | 先 `Modifier::resize` |
| 地图全 -1 | 未收到 static_layer 或分辨率不对 |
| Follow grid 当 OccupancyGrid 订阅 | 类型不同 |

### 延伸阅读

[10 Perception · 障碍](../10_Perception/08_obstacle_perception.md) · [§9](09_survey.md)
