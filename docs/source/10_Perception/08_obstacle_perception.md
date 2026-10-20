# 8. 障碍感知与代价地图

调导航避障、RViz 上看 costmap，或区分「激光层」与「视觉 Follow 栅格」。

车上默认可用的障碍路径：`map/costmap_2d` 的 **ObstacleLayer / VoxelLayer / InflationLayer**。源码：`autonomy/map/costmap_2d/layers/`。

视觉 Follow 的 GridMap **不经过** 本页管线（[§11](11_follow.md)）。

```text
观测 → ObservationBuffer → ObstacleLayer / VoxelLayer → InflationLayer → Costmap2D → Planning
```

| 传感器 | 层 |
|--------|-----|
| 2D 激光 | ObstacleLayer |
| 3D 点云 | VoxelLayer |
| 仅静态地图 | StaticLayer + 可选 obstacle |

### ObstacleLayer（2D 激光）

| 功能 | 说明 |
|------|------|
| Marking | 命中栅格 → LETHAL |
| Clearing | 射线路径 → FREE |
| 高度过滤 | 超范围点忽略 |

```lua
obstacle_layer = {
    observation_sources = "scan",
    scan = {
        topic = "/scan",
        sensor_frame = "laser_link",
        data_type = "LaserScan",
        marking = true, clearing = true,
        obstacle_range = 2.5, raytrace_range = 3.0,
    },
}
```

### VoxelLayer

点云 → 3D 体素 → 投影到 2D；清除/标记逻辑与 ObstacleLayer 同类。

### InflationLayer

在 lethal 周围按距离衰减代价值；参数 `inflation_radius`、`cost_scaling_factor`。公式见 [§3.6](03_math.md)。

### 代价值（Nav2 风格）

| 宏 | 含义 |
|----|------|
| FREE_SPACE (0) | 自由 |
| LETHAL_OBSTACLE (254) | 致命 |
| INSCRIBED_INFLATED_OBSTACLE (253) | 内切 |
| NO_INFORMATION (255) | 未知 |

### 常见坑

| 现象 | 先查 |
|------|------|
| 全图未知 | static_layer、/map |
| 有 scan 无障 | TF、`marking`、range |
| 把 follow grid 当 costmap | 不同话题与类型 |

### 延伸阅读

[07 Map](../07_Map/00_guide.md) · [§6 传感器管线](06_sensor_pipeline.md)
