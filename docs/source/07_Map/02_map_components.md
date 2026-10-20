# 2. 地图组件

日常导航只需弄清 **MapServer、Costmap2D、GridMap** 各喂谁。总览 [00 指南](00_guide.md)，数据流 [01 架构](01_architecture.md)。

先分清 MapServer（静态 OccupancyGrid）、Costmap2D（uint8 避障）、GridMap（float 多层）各喂谁。室内 2D 导航主路径是 Costmap；坡度、跟人等 2.5D 能力走 GridMap + 对应 consumer。

| 组件 | 数据 | 用途 | 文档 |
|------|------|------|------|
| `MapServer` | OccupancyGrid | 加载/发布 `/map` | [01](01_architecture.md) |
| `Costmap2DWrapper` | uint8 栅格 | 规划、避障 | [03](03_costmap2d.md) |
| `GridMapWrapper` | float 多层 | 高程、Follow | [04](04_grid_map.md) |

## 怎么选

- **室内 2D 导航**：Costmap 三层（static + obstacle + inflation）；静态图来自 SLAM 或 MapServer。
- **只看地图、不规划**：MapServer 足够。
- **坡度 / 跟人 / 2.5D**：GridMap + 对应 consumer（如 MPPI Grid critic）。
- **大地图、机器人走远**：Costmap `rolling_window=true`，固定窗口随动。

## 对照

| | MapServer | Costmap2D | GridMap |
|--|-----------|-----------|---------|
| 原点 | 左下 | 左下 | 中心 |
| 数值 | -1/0/100 | 0–255 | float / NaN |
| 数据从哪来 | 文件或回调 | 图层自动 feed | 应用写入 |
| 规划怎么用 | 经 static_layer | 直接读快照 | 需桥接或专用 critic |

Costmap 与 GridMap **无自动桥接**；联调时注意原点与无效值语义不同。
