# 3. Costmap2D 代价地图

2D `uint8` 代价栅格，语义对齐 nav2_costmap_2d。规划与多数控制器读的是这一层，不是原始 OccupancyGrid。架构见 [01 §1.2](01_architecture.md)。 static + obstacle + inflation 是 planner/controller 的默认 uchar 输入；rolling window 解决大地图随动。障碍不刷新或全局拒路径时，先查 layer 插件与观测 topic，而不是改搜索算法。

## 用途

全局/局部路径搜索、激光动态障碍、静态图融合、footprint 膨胀。地形高程、坡度请用 [GridMap](04_grid_map.md) 并在应用层映射，Costmap 没有 elevation 图层。

## 结构

```text
Costmap2DWrapper → LayeredCostmap → plugins_ → combined_costmap_
                  → TF / footprint / 后台 updateMap 线程
```

无 filter 时插件直接写 `combined`；有 filter 时经 `primary` 再过滤。每次更新在窗口内 `resetMap` 后按插件顺序写入。

## 用法

**推荐**：由 `PlannerServer` 配置里的 `costmap` 块创建并 `Start()`，Control 通过 `SetSharedCostmap` 共用。

**独立调试**：

```cpp
auto costmap = std::make_shared<Costmap2DWrapper>(CreateCostmap2DOptions("config"), "global_costmap");
costmap->applyOccupancyGrid(*map); // 或 loadMap(yaml)
costmap->Start();
costmap->feedLaserScan(scan);
// 规划：lock → 复制 getCharMap() → unlock
```

| API | 作用 |
|-----|------|
| `Start` / `Stop` | 后台 `updateMap` |
| `applyOccupancyGrid` / `feedLaserScan` | 静态图 / 激光 |
| `getCostmap` + mutex | 读栅格 |
| `isReady` / `isCurrent` | 是否至少更新一次 / 图层未超时 |

| 模式 | `rolling_window` |
|------|------------------|
| 全局固定图 | `false` |
| 随机器人平移的局部窗 | `true` |

Lua 示例见 [00 指南](00_guide.md)。排障：`isReady`/`isCurrent`、机器人脚下是否为 FREE、RViz 里墙与膨胀带是否合理。

## 坐标

$$
m_x=\lfloor(x-x_0)/\Delta\rfloor,\quad
x=x_0+(m_x+\tfrac12)\Delta,\quad
\mathrm{idx}=m_y N_x+m_x
$$

原点在**左下角**。`worldToMap` / `EnforceBounds` / `NoBounds` 三种边界策略。

## 代价值

| 常量 | 值 | 含义 |
|------|-----|------|
| FREE | 0 | 可通行 |
| 梯度 | 1–252 | 离障碍越近越高 |
| INSCRIBED | 253 | 内切圆碰障 |
| LETHAL | 254 | 致命 |
| UNKNOWN | 255 | 未知 |

规划常把 253/254 当不可通行；`allow_unknown` 由 planner 插件决定。

## 膨胀

$$
c(d)=\begin{cases}
254 & d=0\\
253 & d\cdot\Delta\le r_i\\
\lfloor 252\,e^{-\lambda(d\cdot\Delta-r_i)}\rfloor & \text{else}
\end{cases}
$$

`inflation_radius` 越大越安全、窄通道越难过；`cost_scaling_factor` 越大衰减越快、路径越贴障。

## 图层

| 层 | 要点 |
|----|------|
| Static | 订阅 `/map` 或 load；Trinary / Scale |
| Obstacle | marking → LETHAL；Bresenham clearing；可 footprint 清除 |
| Voxel | 3D 观测投影 2D |
| Denoise | 去孤立噪点 |
| Filters | Keepout / Speed / Binary 等 |

## Rolling 窗口

原点随机器人：$x_0=x_r-W/2$；`updateOrigin` 拷贝重叠区，移出窗口的格恢复 `default_value`。

## Footprint

多边形或 `robot_radius`；内切半径 $r_i$ 对应 INSCRIBED。与 `inflation_radius` 一起决定「能不能过门洞」。

## 常见坑

| 现象 | 先查 |
|------|------|
| `PlannerTimedOut` | `update_frequency`；`isCurrent()` |
| 穿墙 | static 是否加载；obstacle 是否 marking |
| 脚下 LETHAL | `footprint_clearing_enabled`、激光高度 |
| 全 255 | MapServer 未发图；static_layer 未启用 |
