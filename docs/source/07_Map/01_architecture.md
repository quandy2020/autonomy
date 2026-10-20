# 1. Map 模块架构

地图域分三条并行管线：静态占据发布、2D 代价融合、2.5D 多层栅格。Costmap 是 planning/control 的主输入；GridMap 不自动并进 Costmap。

## 三条管线

| 管线 | 入口 | 输出 | 消费者 |
|------|------|------|--------|
| 静态地图 | `MapServer` | OccupancyGrid | `static_layer`、RViz |
| 2D 代价 | `Costmap2DWrapper` | uint8 0–255 | `PlannerServer`、控制器 |
| 2.5D | `GridMapWrapper` | float 多层 | Follow / MPPI 等应用 |

```text
YAML+PGM → MapServer → OccupancyGrid → static_layer ─┐
/scan ──────────────────────────► obstacle_layer ──────┼─► Costmap2D → Planning
inflation_layer ───────────────────────────────────────┘
点云 / 应用 ──► GridMapWrapper → elevation / slope …
```

## Costmap2D

```text
Costmap2DWrapper
├── LayeredCostmap（plugins_ / filters_ / combined_costmap_）
├── TF Buffer、footprint_
└── map_update_thread_ ← Start() 后 updateMap + publishMap
```

**生命周期**：构造 `init()` → `Start()` 开后台线程 → `Pause`/`Stop`。`isReady()`：至少完成一次更新；`isCurrent()`：各图层观测未超时。

**单次 `updateMap`**：取位姿 → rolling 则平移 origin → 各 plugin `updateBounds` → 合并窗口 → `resetMap` → 按序 `updateCosts`（常 `updateWithMax`）→ 可选 filter 链。

| 顺序 | 图层 | 写出 |
|------|------|------|
| 1 | static | LETHAL / FREE |
| 2 | obstacle | marking / clearing |
| 3 | inflation | 1–253 梯度 |

**入数据**：`applyOccupancyGrid` / `loadMap` → Static；`feedLaserScan` / `feedPointCloud2` → ObservationBuffer；TF → 机器人位姿。

**规划读法**：加锁复制 `getCharMap()`，搜索过程不长期持锁。

## GridMap

`GridMapWrapper` 持有 `grid_map::GridMap`（多层 `Eigen::MatrixXf`，中心原点，`move()` 循环 buffer）。无内置 `feed*`；由 perception 或应用写层。

## 线程

| 线程 | 职责 |
|------|------|
| map_update | 周期 `updateMap` |
| 规划 / 控制 | 快照复制后只读 |
| 传感器回调 | 写入 ObservationBuffer |

自定义图层：继承 `CostmapLayer`，实现 `updateBounds`/`updateCosts`，在 conf 的 `plugins` 注册。详见 [03 Costmap2D](03_costmap2d.md)、[04 GridMap](04_grid_map.md)。
