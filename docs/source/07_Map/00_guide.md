# 0. Map 指南

## 背景

规划超时、穿墙、脚下全是障碍——很多不是 planner 坏了，而是 **Costmap2D 图层、膨胀或坐标约定** 不对。本章讲静态栅格、Costmap2D（规划/避障）、GridMap（2.5D）；实现思路接近 nav2_costmap_2d 与 ETH grid_map。

## 思路

日常流水线：

```text
SLAM / 静态地图 → MapServer → Costmap2D → Planning / Control
GridMap（高程等）可并行，不替代 2D costmap
```

- **规划真正吃的是 Costmap2D**：静态图 `static_layer`，激光 `obstacle_layer`，膨胀 `inflation_layer`。  
- **坐标系**：Costmap2D 原点在左下角；GridMap 常以中心为原点（见 [03](03_costmap2d.md) / [04](04_grid_map.md)）。  
- **代价值**：0 自由，254 致命，255 未知。  
- 全局 costmap 常写在 planner 侧 conf；改一处要确认进程是否加载同一份。


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 架构](01_architecture.md) | MapServer / Costmap / GridMap 边界 |
| [02 组件](02_map_components.md) | 模块清单 |
| [03 Costmap2D](03_costmap2d.md) | 图层、代价值、API（最常翻） |
| [04 GridMap](04_grid_map.md) | 2.5D、与 Follow 的关系 |
| [05 综述](05_survey.md) | 选型与对照 |

**穿墙 / 贴边** → [03](03_costmap2d.md) 膨胀与图层。  
**跟人 / 2.5D** → [04](04_grid_map.md) + [Perception · Follow](../10_Perception/11_follow.md)。  
**静态地图不更新** → MapServer 与 `map_topic`。

### 2. Costmap2D 最短配置

在 planner 配置里典型长这样：

```lua
costmap = {
    enabled = true, frame_id = "map", resolution = 0.05,
    width = 20.0, height = 20.0, update_frequency = 5.0,
    robot_radius = 0.22,
    plugins = {"static_layer", "obstacle_layer", "inflation_layer"},
    static_layer = { enabled = true, map_topic = "map" },
    obstacle_layer = {
        enabled = true,
        sensor_sources = {
            scan = { topic = "scan", data_type = "LaserScan", marking = true, clearing = true },
        },
    },
    inflation_layer = { enabled = true, inflation_radius = 0.35, cost_scaling_factor = 3.0 },
}
```

```cpp
auto costmap = std::make_shared<Costmap2DWrapper>(CreateCostmap2DOptions("config"), "global_costmap");
costmap->Start();
```

验证：`HasStaticMap()` / `isCurrent()`；自由格约 0、障碍约 254。

### 3. 常见坑

| 现象 | 先查 |
|------|------|
| `PlannerTimedOut` | `update_frequency`、`isCurrent()` |
| 穿墙 | `inflation_radius`、图层是否启用 |
| 脚下为障 | `footprint_clearing_enabled` |
| 全未知 | MapServer 是否在发 `/map` |
| GridMap 全 NaN | 是否有人写入 elevation；`move()` 后 buffer |
