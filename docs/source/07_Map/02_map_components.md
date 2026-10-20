# 2. 地图组件总览

上手 [§0](00_guide.md)；架构 [§1](01_architecture.md)；选型 [§5](05_survey.md)。

### 2.1 一览

| 组件 | 类型 | 用途 | 文档 |
|------|------|------|------|
| `MapServer` | OccupancyGrid | 加载 / 发布 `/map` | [§0](00_guide.md) |
| `Costmap2DWrapper` | uint8 | 规划避障 | [§3](03_costmap2d.md) |
| `GridMapWrapper` | float 多层 | 地形 / 高程 | [§4](04_grid_map.md) |

Costmap 与 GridMap **当前未桥接**。

### 2.2 对比

| 维度 | MapServer | Costmap2D | GridMap |
|------|-----------|-----------|---------|
| 原点 | 左下 | 左下 | **中心** |
| 数值 | -1/0/100 | 0–255 | float / NaN |
| 传感器 | 文件/回调 | 自动 feed | 应用写入 |
| 规划 | 经 static_layer | 直接读 | 需桥接 |

### 2.3 选型

| 场景 | 推荐 |
|------|------|
| 室内导航 | Costmap：static + obstacle + inflation |
| 仅静态可视化 | MapServer |
| 地形 / 坡度 | GridMap |
| 大地图局部 | Costmap `rolling_window=true` |

### 2.4 数据流

见 [§1.1](01_architecture.md)。专题：[Costmap2D](03_costmap2d.md) · [GridMap](04_grid_map.md) · [综述](05_survey.md)。
