# 3. Costmap2D 代价地图

2D `uint8` 代价栅格（nav2_costmap_2d）。架构见 [§1.2](01_architecture.md)。

### 3.1 适用

| 场景 | — |
|------|--|
| 全局/局部规划、激光避障、静态图融合 | — |
| 地形高程 | → [GridMap](04_grid_map.md) |

### 3.2 结构

```
Costmap2DWrapper → LayeredCostmap → plugins_ → combined_costmap_
→ TF / footprint / 后台 updateMap
```

无 filter 时插件直写 `combined`；有 filter 时经 `primary`。窗口内先 `resetMap` 再按序写入。

### 3.3 用法

**经 PlannerServer**（推荐）：配置 `costmap` 块即可。

**独立**：

```cpp
auto costmap = std::make_shared<Costmap2DWrapper>(CreateCostmap2DOptions("config"), "global_costmap");
costmap->applyOccupancyGrid(*map); // 或 loadMap(yaml)
costmap->Start();
costmap->feedLaserScan(scan);
// 规划：lock → copy getCharMap() → unlock
```

| API | 作用 |
|-----|------|
| `Start` / `Stop` | 后台线程 |
| `applyOccupancyGrid` / `feedLaserScan` | 静态 / 激光 |
| `getCostmap` + mutex | 读 |
| `isReady` / `isCurrent` | 就绪检查 |

| 模式 | `rolling_window` |
|------|------------------|
| 全局 | `false` |
| 局部 | `true`（固定 W×H，随机器人平移） |

配置示例见 [§0.2](00_guide.md)。检查：`isReady`/`isCurrent`、脚下 FREE、可视化墙壁/膨胀。

### 3.5 坐标

$$
m_x=\lfloor(x-x_0)/\Delta\rfloor,\quad
x=x_0+(m_x+\tfrac12)\Delta,\quad
\mathrm{idx}=m_y N_x+m_x
$$

原点为**左下角**。`worldToMap` / `EnforceBounds` / `NoBounds`。

### 3.6 代价值

| 常量 | 值 |
|------|-----|
| FREE | 0 |
| 梯度 | 1–252 |
| INSCRIBED | 253 |
| LETHAL | 254 |
| UNKNOWN | 255 |

### 3.8 Inflation

$$
c(d)=\begin{cases}
254 & d=0\\
253 & d\cdot\Delta\le r_i\\
\lfloor 252\,e^{-\lambda(d\cdot\Delta-r_i)}\rfloor & \text{else}
\end{cases}
$$

种子 = LETHAL；按距离等级 BFS 扩展。`inflation_radius` ↑ → 更安全、窄通道更难；`cost_scaling_factor` ↑ → 衰减更快。

### 3.9 图层

| 层 | 要点 |
|----|------|
| Static | Trinary / Scale；可 resize 全局图 |
| Obstacle | marking LETHAL；Bresenham clearing；footprint 清除 |
| Voxel | 3D 投影到 2D |
| Denoise | 过滤孤立簇 |
| Filters | Keepout / Speed / Binary |

### 3.10 Rolling

$x_0=x_r-W/2$；`updateOrigin` 拷贝重叠区，移出格 = `default_value`。

### 3.11 Footprint

多边形或 `robot_radius`；内切 $r_i$ → INSCRIBED。配置见 `planner.lua`。
