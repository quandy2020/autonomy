# 8. 障碍感知与代价地图

当前导航栈障碍路径：`map/costmap_2d`（`ObstacleLayer` / `VoxelLayer` / `InflationLayer`）。源码：`autonomy/map/costmap_2d/layers/`。

### 8.1 管线

```
观测 → ObservationBuffer → ObstacleLayer/VoxelLayer → InflationLayer → Costmap2DWrapper → Planning
```

### 8.2 ObstacleLayer

| 功能 | 说明 |
|------|------|
| Marking | 终点栅格 → `LETHAL_OBSTACLE` |
| Clearing | 射线路径 → `FREE_SPACE` |
| 高度过滤 | $z\notin[\mathrm{min},\mathrm{max}]$ 忽略 |

流程：`LaserScan` → TF → 终点 $p_{end}$ → 清除 + 标记。

```lua
obstacle_layer = {
    observation_sources = "scan",
    scan = {
        topic = "/scan",
        sensor_frame = "laser_link",
        data_type = "LaserScan",
        marking = true, clearing = true,
        obstacle_range = 2.5, raytrace_range = 3.0,
        max_obstacle_height = 2.0, min_obstacle_height = 0.0,
    },
}
```

### 8.3 VoxelLayer

点云 → 3D 体素 → 投影 2D costmap；清除/标记同 ObstacleLayer。

### 8.4 InflationLayer

$$
f(d)=\begin{cases}
\mathrm{LETHAL}&d\le r_{robot}\\
\mathrm{INSCRIBED}\,e^{-\lambda(d-r_{robot})}&r_{robot}<d\le r_{infl}\\
0&d>r_{infl}
\end{cases}
$$

```lua
inflation_layer = {
    inflation_radius = 0.55,
    cost_scaling_factor = 10.0,
}
```

### 8.5 代价值

| 值 | 宏 | 含义 |
|----|-----|------|
| 0 | `FREE_SPACE` | 自由 |
| 1–252 | — | 梯度 |
| 253 | `INSCRIBED_INFLATED_OBSTACLE` | 内切边界 |
| 254 | `LETHAL_OBSTACLE` | 致命 |
| 255 | `NO_INFORMATION` | 未知 |

### 8.6 接口与调试

| API / 工具 | 用途 |
|------------|------|
| `getCost` / `isCollisionFree` / `updateMap` | Planning/Control 查询 |
| RViz costmap | 可视化 |
| 无障碍 / 全未知 | 查 `/scan`、TF、static_layer |

### 8.7 与 Perception 融合（规划）

| 策略 | 说明 |
|------|------|
| 叠加 | max cost |
| 独立层 | `SemanticLayer` 并列 |

### 8.8 参见

- [§3.6](03_math.md) · [07 Map](../07_Map/00_guide.md) · [§6](06_sensor_pipeline.md)
