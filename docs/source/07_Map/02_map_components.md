# 2. 地图组件总览

`autonomy/map` 提供三条并行管线：**MapServer**（静态占据栅格）、**Costmap2D**（规划/避障代价图）、**GridMap**（2.5D 多层浮点栅格）。本文是 **§3–§4 专题的索引与对比**；上手见 [§0](00_guide.md)，架构见 [§1](01_architecture.md)，谱系与选型见 [§5 综述](05_survey.md)。

---

## 2.1 组件一览

| 组件 | 文档 | 数据类型 | 典型用途 | 状态 |
|------|------|----------|----------|------|
| `MapServer` | [§0.5](00_guide.md#05-配置与-api) | `OccupancyGrid` (-1/0/100) | 加载 SLAM 地图、发布 `/map` | ✅ |
| `Costmap2DWrapper` | [§3 Costmap2D](03_costmap2d.md) | `uint8` 0–255 | 全局/局部规划、碰撞检测 | ✅ |
| `GridMapWrapper` | [§4 GridMap](04_grid_map.md) | `float` 多层矩阵 | 地形高程、坡度、多模态感知 | ✅ |

三者通过 `MapInterface`（`Start` / `Stop` / `Pause` / `Resume`）管理；**Costmap2D 与 GridMap 当前未桥接**，联合使用需在应用层处理坐标与数值映射。

---

## 2.2 对比矩阵

| 维度 | MapServer | Costmap2D | GridMap |
|------|-----------|-----------|---------|
| 原点 | 左下角 | 左下角 | **地图中心** |
| 数值 | 离散 -1/0/100 | 离散 0–255 | 连续 float / `NaN` |
| 传感器 | 无（读文件/回调） | 自动 `feedLaserScan` 等 | **应用层写入** |
| 更新 | 低频 / 一次性 | 后台线程 1–20 Hz | 手动 `move()` + 写层 |
| 规划消费 | 经 `static_layer` | `PlannerServer` 直接读 | 需桥接或自定义 |
| 对标 | nav2_map_server | nav2_costmap_2d | ETH grid_map |

坐标差异与互转注意点见 [§0.3](00_guide.md#03-环境形式化) · [§3.5](03_costmap2d.md#35-坐标变换) · [§4.6](04_grid_map.md#46-坐标变换)。

---

## 2.3 选型速查

| 场景 | 推荐 | 关键配置 |
|------|------|----------|
| 标准室内导航 | Costmap2D | `static_layer` + `obstacle_layer` + `inflation_layer` |
| 仅静态地图可视化 | MapServer | YAML+PGM |
| 地形 / 高程 / 坡度 | GridMap | `elevation` 层 + 应用层点云投影 |
| 大地图局部规划 | Costmap2D `rolling_window=true` | 见 [§3.10](03_costmap2d.md#310-rolling-window) |
| Costmap + 高程联合 | GridMap + 应用桥接 | 见 [§4.3.7](04_grid_map.md#437-与-costmap2d-联合使用应用层桥接) |

完整矩阵与决策树见 [§5.10](05_survey.md#510-autonomy-map-模块定位)。

---

## 2.4 公共数据流

```
YAML+PGM ──► MapServer ──► OccupancyGrid ──► static_layer ──┐
/scan ──────────────────────────────► obstacle_layer ─────────┼──► Costmap2D ──► Planning
                                                            │
                                              inflation_layer ┘

点云 / 深度 ──► GridMapWrapper（应用写入）──► elevation / slope / ...
                      ✗ 当前未自动桥接到 Costmap2D
```

运行时序与线程模型见 [§1.3.4](01_architecture.md#134-单次更新数据如何流过各层) · [§1.11](01_architecture.md#111-线程与并发)。

---

## 2.5 扩展阅读

| § | 文档 | 内容 |
|---|------|------|
| 3 | [Costmap2D](03_costmap2d.md) | 图层、膨胀、滚动窗口、配置 |
| 4 | [GridMap](04_grid_map.md) | 2.5D 层、buffer、插值、桥接 |
| 5 | [综述](05_survey.md) | 历史、分类、业界生态 |
