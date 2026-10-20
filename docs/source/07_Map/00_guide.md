(map-guide)=
# 0. Map 地图模块指南

`autonomy/map`：静态 OccupancyGrid、Costmap2D（规划/避障）、GridMap（2.5D 地形）。对齐 nav2_costmap_2d 与 ETH grid_map。

流水线：SLAM / 静态地图 → MapServer → Costmap2D → Planning / Control；GridMap 2.5D 并行。

---

## 0.2 快速开始

**Costmap2D**（经 PlannerServer）：在 `planner.lua` 配 `costmap` 块；`autonomy.lua` 挂 `planning`；MapServer 发 `/map` 或 `static_layer` 订阅。

```lua
costmap = {
    enabled = true, frame_id = "map", resolution = 0.05,
    width = 20.0, height = 20.0, update_frequency = 5.0,
    robot_radius = 0.22,
    plugins = {"static_layer", "obstacle_layer", "inflation_layer"},
    static_layer = { enabled = true, map_topic = "map" },
    obstacle_layer = {
        enabled = true,
        sensor_sources = { scan = { topic = "scan", data_type = "LaserScan", marking = true, clearing = true } },
    },
    inflation_layer = { enabled = true, inflation_radius = 0.35, cost_scaling_factor = 3.0 },
}
```

```cpp
auto costmap = std::make_shared<Costmap2DWrapper>(CreateCostmap2DOptions("config"), "global_costmap");
costmap->Start();
```

**MapServer** / **GridMap**：`MapServer::Start()`；`GridMapWrapper` 由应用层写 `elevation`（无自动传感器管线）。

验证：`HasStaticMap()` · `isCurrent()` · 自由≈0、障碍≈254。

---

## 0.3 坐标与代价值

| — | Costmap2D | GridMap |
|--|-----------|---------|
| 原点 | 左下角 | **中心** |
| 数值 | uint8 0–255 | float / NaN |

| 值 | 含义 |
|----|------|
| 0 | FREE |
| 1–252 | 膨胀梯度 |
| 253 / 254 | INSCRIBED / LETHAL |
| 255 | UNKNOWN |

公式：[§3.5–§3.8](03_costmap2d.md) · [§4.6](04_grid_map.md)。

---

## 0.5 API 要点

| API | 时机 |
|-----|------|
| `Start()` / `feedLaserScan` / `applyOccupancyGrid` | 启动与传感器 |
| `getCostmap()` + lock 复制 | 规划前 |
| `isReady()` / `isCurrent()` | 规划前检查 |

自定义图层：继承 `CostmapLayer`，`.so` + `plugins` 列表。排错见下。

---

## 0.7 故障排查

| 现象 | 处理 |
|------|------|
| `PlannerTimedOut` | `update_frequency`、`isCurrent()` |
| 穿墙 | 增大 `inflation_radius` |
| 脚下为障 | `footprint_clearing_enabled` |
| 全未知 | MapServer / `map_topic` |
| 坐标偏 | `frame_id` / TF |
| GridMap NaN | 写入与 `move()` 后 buffer |

性能：全局 ≤2048² · 更新 ~5 Hz · 规划复制 char map。
