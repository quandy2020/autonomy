(planning-architecture)=
# 1. Planning 模块架构

Planning 在 **Costmap2D 快照** 上算全局几何路径，交给 Control 跟踪。上手 [00 指南](00_guide.md)；算法 [02–05](02_planner_algorithms.md)。

## 在栈中的位置

```text
Navigator → PlannerServer → GlobalPlanner 插件
              ↑ Costmap2D 快照（复制后搜索）
              ↓
           Path → ControllerServer
```

设计约束：Nav2 风格插件、Lua→`PlannerOptions`、规划前复制 costmap 避免长时间持锁。

## 分层

| 层 | 组件 |
|----|------|
| 服务 | `PlannerServer`：`GetPlan` / `IsPathValid`、TF、结果码 |
| 算法 | `GlobalPlanner::CreatePlan` |
| 地图 | `Costmap2DWrapper`（可与 Control 共享） |
| 后处理 | PathSimplifier / SimpleSmoother（可选） |

**NavFn 约定**：用户语义「起点/终点」在 API 层传入；内部 `setGoal` 对应用户起点、`setStart` 对应用户终点，再 `calcPath` 从终点势场梯度回到起点。

## GetPlan 流程

TF 变换起终点 → 选插件 → 复制 costmap → `CreatePlan` → 可选简化/平滑 → 返回 `Path` 或抛结果码异常。

## 插件 ID

| ID | 类 |
|----|-----|
| `navfn_planner` | NavfnPlanner |
| `dijkstra_planner` | DijkstraPlanner |
| `theta_star_planner` | ThetaStarPlanner |

加载：`planner.lua` → PluginManager → `CreateInstance`。`default_planner_id` 须在 `planner_plugins` 列表中。

## 结果码

| 码 | 含义 |
|----|------|
| 0 | SUCCESS |
| 51 | CANCELED |
| 54/55 | Start/Goal occupied |
| 56 | NO_PATH |
| 57 | TIMEOUT |
| 59 | TF_ERROR |
| 61 | INVALID_PLUGIN |

搜索循环内通过 `cancel_checker` 响应取消；不在持锁状态下做长搜索。

## 扩展

继承 `GlobalPlanner`，实现 `CreatePlan`，`AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN`，在 lua 增加 id。
