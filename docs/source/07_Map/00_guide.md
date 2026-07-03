(map-guide)=
# 0. Map 地图模块指南

`autonomy/map`：环境地图子系统，对齐 nav2 `nav2_costmap_2d` 与 ETH `grid_map`。输出 `OccupancyGrid`（静态）、`Costmap2D`（规划/避障）、`GridMap`（地形/高程）。

| 本文 §0 | 其他文档 |
|---------|----------|
| 上手、配置、排错 | [§1 架构](01_architecture.md) · [§2 组件](02_map_components.md) · [§5 综述](05_survey.md) |

<div class="nav-costmap-banner">
  <strong>地图在导航栈中的位置</strong>
  <span class="nav-costmap-detail">SLAM / 静态地图 → MapServer → Costmap2D → Planning / Control</span>
  <span class="nav-costmap-arrow">GridMap 2.5D 独立并行 →</span>
</div>

---

## 0.1 文档地图

| 角色 | 阅读顺序 |
|------|----------|
| 新手 | §0.2 → [§1](01_architecture.md) → [§2](02_map_components.md) |
| 集成 | §0.2、§0.5、§0.7 → [§3 图层](03_costmap2d.md#39-图层详解) |
| 算法 | [§5](05_survey.md) → [§2](02_map_components.md) → [§3–§4](03_costmap2d.md) |

| § | 文档 | 内容 |
|---|------|------|
| 0 | 本指南 | 上手、形式化、配置、排错 |
| 1 | [架构](01_architecture.md) | 三条管线、数据流、线程 |
| 2 | [组件总览](02_map_components.md) | MapServer / Costmap2D / GridMap 对比 |
| 3–4 | Costmap2D · GridMap | 实现专题 |
| 5 | [综述](05_survey.md) | 历史、分类、选型 |

---

## 0.2 快速开始

### Costmap2D（三步）

1. `config/planner/planner.lua` — 配置 `costmap` 块（图层、分辨率、footprint）
2. `config/autonomy.lua` — `planning = AUTONOMY_PLANNER`（`PlannerServer` 自动构造 `Costmap2DWrapper`）
3. 确保 `MapServer` 发布静态地图，或 `static_layer` 订阅 `/map`

```lua
-- config/planner/planner.lua
costmap = {
    enabled = true,
    name = "global_map",
    frame_id = "map",
    resolution = 0.05,
    width = 20.0,
    height = 20.0,
    update_frequency = 5.0,
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
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
auto costmap = std::make_shared<autonomy::map::costmap_2d::Costmap2DWrapper>(
    autonomy::map::CreateCostmap2DOptions("config"), "global_costmap");
costmap->Start();
costmap->updateRobotPose(x, y, yaw);
```

### MapServer

```cpp
auto server = std::make_shared<autonomy::map::MapServer>(
    autonomy::map::LoadOptions("config").map_server());
server->SetMapPublishCallback([](const auto& grid) { /* → static_layer */ });
server->Start();
```

### GridMap

```cpp
auto grid = std::make_shared<autonomy::map::grid_map::GridMapWrapper>(
    autonomy::map::CreateGridMapOptions("config"));
grid->Start();
// 应用层写入 elevation；查询见 §4
```

**验证**：`MapServer::HasStaticMap()` · `costmap->isCurrent()` · 自由区 ≈ 0、障碍 ≈ 254。

运行时序见 [§1.3.4](01_architecture.md#134-单次更新数据如何流过各层)；组件对比见 [§2](02_map_components.md)。

---

## 0.3 环境形式化

(map-overview)=

平面环境 $\mathcal{W} \subset \mathbb{R}^2$ 离散为栅格 $\mathcal{M} = \{ c_{ij} \}$，分辨率 $\Delta$ 将 $\mathbf{p}=(x,y)^\top$ 映射到索引 $(m_x, m_y)$。Autonomy 提供两套坐标约定：

| 属性 | Costmap2D | GridMap |
|------|-----------|---------|
| 原点 | 地图**左下角** $(x_0, y_0)$ | 地图**中心** $\mathbf{p}_{\text{map}}$ |
| 索引 | $idx = m_y \cdot N_x + m_x$ | Eigen 矩阵 + 循环 buffer |
| 数值 | 离散 0–255 | 连续 float，`NaN` 表无效 |

互转须显式处理原点差异；公式见 [§3.5](03_costmap2d.md#35-坐标变换) · [§4.6](04_grid_map.md#46-坐标变换)。

---

## 0.4 代价值语义

规划读 Costmap2D **快照**；代价值为 Planning / Control 公共输入。

| 值 | 常量 | 含义 |
|----|------|------|
| 0 | `FREE_SPACE` | 完全自由 |
| 1–252 | — | 距障碍越近代价越高 |
| 253 | `INSCRIBED_INFLATED_OBSTACLE` | 内切膨胀区 |
| 254 | `LETHAL_OBSTACLE` | 致命障碍 |
| 255 | `NO_INFORMATION` | 未知 |

膨胀公式与图层见 [§3.6–§3.8](03_costmap2d.md#36-代价值体系)；Planning 侧解读见 [Planning §0.4](../08_Planning/00_guide.md#04-代价地图)。

---

## 0.5 配置与 API

(map-usage)=

### Costmap2D

| 项 | 值 |
|----|-----|
| 配置 | `config/planner/planner.lua` → `costmap` |
| 加载 | `map::CreateCostmap2DOptions("config")` |
| 集成 | `PlannerServer` 自动构造并 `Start()` |

**端到端（经 PlannerServer）**

```
MapServer::Start() → PlannerServer 构造 Costmap2DWrapper
→ static_layer 收地图 → bridge 转发 /scan → TF 就绪 → isReady()
```

| API | 何时 |
|-----|------|
| `Start()` | 系统启动 |
| `applyOccupancyGrid` | 静态地图到达 |
| `feedLaserScan` | 每次激光回调 |
| `getCostmap()` + lock | 规划前复制快照 |
| `isReady()` / `isCurrent()` | 规划前检查 |

图层参数要点见 [§3.9](03_costmap2d.md#39-图层详解)；详细 API 见 [§3.3](03_costmap2d.md#33-详细使用指南)。

### GridMap

| 项 | 值 |
|----|-----|
| Proto | `map_grid_option.proto` |
| 封装 | `GridMapWrapper` |
| 差异 | 无自动传感器管线；应用层投影点云到 `elevation` |

详见 [§4.3](04_grid_map.md#43-详细使用指南)。

### MapServer

| API | 用途 |
|-----|------|
| `Start()` | 加载并发布 |
| `GetStaticMapShared()` | 获取 `OccupancyGrid` |
| `SetMapPublishCallback` | 转发给 `applyOccupancyGrid` |

---

## 0.6 自定义 Costmap 图层

1. 继承 `CostmapLayer` 或 `Layer`，实现 `updateBounds()` / `updateCosts()`
2. 编译为 `.so`，在 `plugins` 列表注册
3. Lua 中添加对应参数块

参考 `autonomy/map/costmap_2d/layers/`；架构见 [§1.12](01_architecture.md#112-扩展自定义图层)。

---

## 0.7 故障排查

| 现象 | 常见原因 | 处理 |
|------|----------|------|
| `PlannerTimedOut` | costmap 未更新 | `update_frequency`、`isCurrent()` |
| 路径穿墙 | 膨胀不足 | 增大 `inflation_radius` |
| 机器人被标为障碍 | footprint 未清除 | `footprint_clearing_enabled=true` |
| 全未知 (255) | 静态图未加载 | MapServer、`static_layer.map_topic` |
| 坐标偏移 | TF / `frame_id` | 统一 frame 与 TF 树 |
| GridMap 查询 NaN | 无数据或越界 | 检查 `move()` 后 buffer |

**检查清单**：`HasStaticMap()` · `isCurrent()` · `worldToMap` 边界 · 可视化 costmap 快照。

性能建议：全局图 ≤ 2048² · 更新 5 Hz · 规划时复制 char map 避免长持锁。详见 [§3.3.6](03_costmap2d.md#336-启动检查清单)。
