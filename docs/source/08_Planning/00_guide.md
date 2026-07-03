(planning-guide)=
# 0. Planning 路径规划指南

`autonomy/planning`：全局路径规划，对齐 nav2 `nav2_planner`。输入起终点 + Costmap2D，输出 `planning_msgs::Path`。

| 本文 §0 | 其他文档 |
|---------|----------|
| 快速开始、配置、排错 | [§1 架构](01_architecture.md) · [§2 规划器](02_planner_algorithms.md) · [§6 综述](06_survey.md) |

---

## 0.1 文档地图

| 角色 | 阅读顺序 |
|------|----------|
| 新手 | §0.2 → [§1](01_architecture.md) → [§2](02_planner_algorithms.md) |
| 集成 | §0.2、§0.5、§0.8 → [§6.6 选型](06_survey.md#663-场景选型矩阵) |
| 算法 | [§6](06_survey.md) → [§2](02_planner_algorithms.md) → [§3–§5](03_navfn.md) |

| § | 文档 | 内容 |
|---|------|------|
| 0 | 本指南 | 上手、配置、排错 |
| 1 | [架构](01_architecture.md) | 分层、GetPlan、插件加载 |
| 2 | [规划器总览](02_planner_algorithms.md) | 三插件对比与索引 |
| 3–5 | NavFn · Dijkstra · Theta* | 算法专题 |
| 6 | [综述](06_survey.md) | 历史、分类、选型 |

---

## 0.2 快速开始

1. `config/planner/planner.lua` — 启用插件、设 `default_planner_id`
2. `config/autonomy.lua` — `planning = AUTONOMY_PLANNER`
3. 启动后 `Autonomy` 构造 `PlannerServer`

```lua
default_planner_id = "navfn_planner"
planner_plugins = { "navfn_planner", "dijkstra_planner", "theta_star_planner" }
```

```cpp
auto server = std::make_shared<PlannerServer>(CreateOptions("config"));
auto path = server->GetPlan(start, goal, "navfn_planner", []{ return false; });
```

运行时序见 [§1.3](01_architecture.md#13-单次规划流程)。插件对比见 [§2](02_planner_algorithms.md)。

---

## 0.3 问题形式化

(planning-overview)=

求 $\tau:[0,1]\to\mathcal{C}_{\mathrm{free}}$，$\tau(0)=q_s$，$\tau(1)=q_g$，最小化路径长度与障碍代价：

$$
J(\tau)=\int_0^1 \big(w_l\|\tau'(s)\|+w_c\,c(\tau(s))\big)\,ds.
$$

离散栅格 $C$ 上由各插件搜索；分类与约束见 [§6.3](06_survey.md#63-算法分类全景)。

---

## 0.4 代价地图

规划读 Costmap2D **快照**；代价值为各插件公共输入。

| 值 | 含义 | NavFn | Theta* |
|----|------|-------|--------|
| 0 | FREE | 通行 | 通行 |
| 1–252 | 梯度 | $F_{ij}=C_n+\kappa_F c_{ij}$ | $\tau(n)\propto c_n/252$ |
| 253 / 254 | INSCRIBED / LETHAL | 阻塞 | LOS 阻塞 |
| 255 | UNKNOWN | `allow_unknown` | `allow_unknown` |

NavFn 参数推导 [§3 NavFn §3.2](03_navfn.md#32-代价映射)；图层与膨胀 [Map · Costmap2D](../07_Map/03_costmap2d.md)、[§1.2.1](01_architecture.md#121-地图层-costmap2dwrapper)。

---

## 0.5 配置与 API

(planning-usage)=

**`config/planner/planner.lua`**

| 字段 | 说明 | 默认建议 |
|------|------|----------|
| `default_planner_id` | 默认插件 | `navfn_planner` |
| `planner_plugins` | 启用列表 | 三插件 |
| `path_simplify_epsilon` | DP 简化，0=关 | `0.0` |
| `auto_smooth_after_plan` | 规划后平滑 | `false` |
| `costmap` | 全局地图子配置 | 见 Map 文档 |

**`PlannerServer`**

| API | 用途 |
|-----|------|
| `GetPlan(…)` | 单次规划 |
| `IsPathValid(…)` | 路径仍可行？ |
| `GetCostmapWrapper()` | 共享 costmap |

节点 `planner_server`。分层、插件加载、扩展见 [§1](01_architecture.md)。

---

## 0.6 路径后处理

由 §0.5 中 `path_simplify_epsilon`、`auto_smooth_after_plan` 控制；默认均关（简化可能使绕障路径坍缩）。流水线位置见 [§1.3](01_architecture.md#13-单次规划流程)。

---

## 0.7 插件扩展

见 [§1.6 扩展自定义规划器](01_architecture.md#16-扩展自定义规划器)。

---

## 0.8 故障排查

| 现象 | 先查 |
|------|------|
| `NoValidPathCouldBeFound` | 连通性、`tolerance`、goal 自由区 |
| `StartOccupied` / `GoalOccupied` | 位姿、膨胀半径 |
| `PlannerTimedOut` | `costmap->isCurrent()`、地图尺寸 |
| `PlannerTFError` | `frame_id`、TF |
| 穿墙 / 贴障 | 膨胀、`footprint` |
| 路径锯齿 | `theta_star_planner` 或后处理 |

结果码表 [§1.5](01_architecture.md#15-错误处理与并发) · 选型 [§6.6](06_survey.md#66-autonomy-内置规划器选型) · 系统排查 [§6.7.3](06_survey.md#673-因素对策速查表)。
