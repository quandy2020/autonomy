(planning-guide)=
# 0. Planning 路径规划指南

`autonomy/planning`：全局路径规划（nav2_planner）。输入起终点 + Costmap2D，输出 `Path`。

---

## 0.2 快速开始

```lua
default_planner_id = "navfn_planner"
planner_plugins = { "navfn_planner", "dijkstra_planner", "theta_star_planner" }
```

```cpp
auto server = std::make_shared<PlannerServer>(CreateOptions("config"));
auto path = server->GetPlan(start, goal, "navfn_planner", []{ return false; });
```

时序 [§1.3](01_architecture.md)；对比 [§2](02_planner_algorithms.md)。

---

## 0.3 形式化

$$
J(\tau)=\int_0^1\big(w_l\|\tau'(s)\|+w_c\,c(\tau(s))\big)\,ds
$$

分类见 [§6](06_survey.md)。

---

## 0.4 代价地图

| 值 | NavFn | Theta* |
|----|-------|--------|
| 0 | 通行 | 通行 |
| 1–252 | $F=C_n+\kappa_F c$ | $\tau\propto c/252$ |
| 253/254 | 阻塞 | LOS 阻塞 |
| 255 | `allow_unknown` | 同左 |

详见 [Map · Costmap](../07_Map/03_costmap2d.md)。

---

## 0.5 配置与 API

| 字段 | 说明 |
|------|------|
| `default_planner_id` | 默认插件 |
| `planner_plugins` | 启用列表 |
| `path_simplify_epsilon` / `auto_smooth_after_plan` | 后处理（默认关） |
| `costmap` | 全局图 |

| API | 用途 |
|-----|------|
| `GetPlan` / `IsPathValid` | 规划 / 校验 |
| `GetCostmapWrapper` | 共享 costmap |

---

## 0.8 故障排查

| 现象 | 先查 |
|------|------|
| `NoValidPathCouldBeFound` | 连通、`tolerance`、goal 自由 |
| `Start/GoalOccupied` | 位姿、膨胀 |
| `PlannerTimedOut` | `isCurrent()`、地图尺寸 |
| `PlannerTFError` | frame / TF |
| 锯齿 | `theta_star` 或后处理 |

结果码 [§1.5](01_architecture.md)；选型 [§6](06_survey.md)。
