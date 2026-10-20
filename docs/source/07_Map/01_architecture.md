# 1. Map 模块架构

### 1.1 三条管线

| 管线 | 入口 | 输出 | 消费者 |
|------|------|------|--------|
| 静态地图 | `MapServer` | OccupancyGrid | `static_layer`、可视化 |
| 2D 代价 | `Costmap2DWrapper` | uint8 0–255 | `PlannerServer` |
| 2.5D | `GridMapWrapper` | float 多层 | 应用层（未自动桥接 Costmap） |

```
YAML+PGM → MapServer → OccupancyGrid → static_layer ─┐
/scan ──────────────────────────► obstacle_layer ──────┼─► Costmap2D → Planning
inflation_layer ┘
点云 → GridMapWrapper（应用写入）→ elevation / slope
```

---

### 1.2 Costmap2D

```
Costmap2DWrapper
├── LayeredCostmap（plugins_ / filters_ / combined_costmap_）
├── TF Buffer、footprint_
└── map_update_thread_ ← Start() 后 updateMap + publishMap
```

**生命周期**：构造 `init()` → `Start()` 后台更新 → `Pause`/`Stop`。`isReady()`：至少一次更新；`isCurrent()`：图层未超时。

**单次 `updateMap`**：

1. 取位姿；rolling 则居中平移 origin 
2. 各 plugin `updateBounds` → 合并窗口 → `resetMap` 
3. 按序 `updateCosts`（常 `updateWithMax`） 
4. 有 filter：primary → combined → filter 

| 顺序 | 图层 | 写出 |
|------|------|------|
| 1 | static | LETHAL / FREE |
| 2 | obstacle | marking / clearing |
| 3 | inflation | 1–253 梯度 |

**入数据**：`applyOccupancyGrid` / `loadMap` → Static；`feedLaserScan` / `feedPointCloud2` → ObservationBuffer；TF → 位姿。

**规划读**：加锁复制 `getCharMap()`，搜索不持锁。

---

### 1.3 GridMap

`GridMapWrapper` → `grid_map::GridMap`（多层 `Eigen::MatrixXf`，中心原点，`move()` 循环 buffer）。无自动 `feed*`；应用层投影点云写层。

---

### 1.4 线程

| 线程 | 职责 |
|------|------|
| map_update | 周期 `updateMap` |
| 规划 / 控制 | 快照复制后只读 |
| 传感器回调 | 写入 ObservationBuffer |

扩展图层：继承 `CostmapLayer`，实现 `updateBounds`/`updateCosts`，注册 `plugins`。详见 [§3](03_costmap2d.md) · [§4](04_grid_map.md)。
