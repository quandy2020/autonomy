# Map 地图模块指南

`autonomy/map` 是 Autonomy 的**环境地图**子系统，对齐 ROS 2 Navigation2 `nav2_costmap_2d` 与 ETH `grid_map`。本文档为模块总入口，以下按 **§1–§8** 顺序组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§5 架构](05_architecture.md) → [§4 使用指南](04_usage.md) |
| 算法研发 | [§8 综述](08_survey.md) → [§3 数学原理](03_math.md) → [§6 Costmap2D](06_costmap2d.md) / [§7 GridMap](07_grid_map.md) |
| 集成调试 | [§2 快速开始](02_quickstart.md) → [§4 使用指南](04_usage.md) → [§4.5 排错](04_usage.md#45-故障排查) → [§6 图层配置](06_costmap2d.md#69-图层详解) |

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 模块概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_math.md](03_math.md) | 数学原理 |
| 4 | [04_usage.md](04_usage.md) | 使用指南 |
| 5 | [05_architecture.md](05_architecture.md) | 模块架构设计 |
| 6 | [06_costmap2d.md](06_costmap2d.md) | Costmap2D 代价地图 |
| 7 | [07_grid_map.md](07_grid_map.md) | GridMap 2.5D 栅格地图 |
| 8 | [08_survey.md](08_survey.md) | 地图表示综述 |

<div class="nav-costmap-banner">
  <strong>地图在导航栈中的位置</strong>
  <span class="nav-costmap-detail">SLAM / 静态地图 → MapServer → Costmap2D → Planning / Control</span>
  <span class="nav-costmap-arrow">GridMap 2.5D 独立并行 →</span>
</div>
