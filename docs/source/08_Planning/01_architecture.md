(planning-architecture)=
# 1. Planning 模块架构

上手 [§0](00_guide.md)；算法 [§2–§5](02_planner_algorithms.md)。

### 1.1 位置

```
Navigator → PlannerServer → GlobalPlanner 插件
│ │
│ Costmap2D 快照
▼
Path → Control
```

约束：nav2 对齐 · 插件化 · Lua→PlannerOptions · 规划前复制快照。

### 1.2 分层

| 层 | 组件 |
|----|------|
| 服务 | `PlannerServer`：`GetPlan` / `IsPathValid`、TF、结果码 |
| 算法 | `GlobalPlanner::CreatePlan` |
| 地图 | `Costmap2DWrapper`（共享；插件内 lock→copy） |
| 后处理 | PathSimplifier / SimpleSmoother（可选） |

NavFn 约定：用户起点 → 代码 `setGoal`；用户终点 → `setStart`；`calcPath` 自终点梯度回起点。

### 1.3 GetPlan 流程

TF → CreatePlan（复制 costmap → 搜索）→ 可选简化/平滑 → Path 或异常。

### 1.4 插件

| ID | 类 |
|----|-----|
| `navfn_planner` | NavfnPlanner |
| `dijkstra_planner` | DijkstraPlanner |
| `theta_star_planner` | ThetaStarPlanner |

加载：`planner.lua` → PluginManager → `CreateInstance`。

### 1.5 结果码

| 码 | 含义 |
|----|------|
| 0 | SUCCESS |
| 51 | CANCELED |
| 54/55 | Start/Goal occupied |
| 56 | NO_PATH |
| 57 | TIMEOUT |
| 59 | TF_ERROR |
| 61 | INVALID_PLUGIN |

并发：搜索不持 costmap 锁；`cancel_checker` 周期检查。

### 1.6 扩展

继承 `GlobalPlanner`，实现 `CreatePlan`，`AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN`，在 lua 注册。
