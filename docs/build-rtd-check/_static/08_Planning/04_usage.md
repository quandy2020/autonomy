(planning-usage)=
# 4. 使用指南

### 4.1 配置

**入口文件**：`config/planner/planner.lua` → `config/autonomy.lua`

```lua
AUTONOMY = {
    planning = AUTONOMY_PLANNER,
}
```

**关键字段**：

| 字段 | 说明 | 默认建议 |
|------|------|----------|
| `default_planner_id` | 默认规划器 | `navfn_planner` |
| `planner_plugins` | 启用的插件列表 | 三个全启 |
| `path_simplify_epsilon` | DP 简化阈值，0=禁用 | `0.0` |
| `auto_smooth_after_plan` | 规划后自动平滑 | `false` |
| `costmap` | 全局代价地图 | 见 `planner.lua` |

完整配置示例见 `config/planner/planner.lua`。C++ 加载：

```cpp
auto options = autonomy::planning::CreateOptions("config");
```

### 4.2 PlannerServer API

| API | 用途 |
|-----|------|
| `GetPlan(start, goal, id, cancel)` | 单次规划，返回 `Path` |
| `IsPathValid(path, max_cost, unknown)` | 路径碰撞检测 |
| `SetPathUpdateCallback(fn)` | 规划成功后的回调 |
| `GetCostmapWrapper()` | 获取共享 costmap |

**选择规划器**：

| 调用 | 行为 |
|------|------|
| `GetPlan(..., "navfn_planner", ...)` | 指定插件 |
| `GetPlan(..., "", ...)` | 仅 1 个插件时自动选择 |
| 多插件且 id 无效 | 抛出 `InvalidPlanner` |

**异常处理**：`NoValidPathCouldBeFound`、`PlannerTimedOut`、`PlannerTFError` 等，见 [§4.8](#48-故障排查)。

### 4.3 系统集成

```cpp
// system::Autonomy 自动创建
planner_ = std::make_shared<planning::PlannerServer>(options_.planner_options());
```

```
用户目标 → Navigator BT → GetPlan() → Path → Controller
                              ↓
                        IsPathValid() → 失效则重规划
```

### 4.4 通信接口

| 类型 | 名称 | 说明 |
|------|------|------|
| 节点 | `planner_server` | autolink 节点 |
| 代价地图 | `/global_costmap` | 全局地图话题 |
| 服务 | `is_path_valid` | 路径有效性 |
| Action | `compute_path_to_pose` | Navigator 封装 |

### 4.5 路径后处理

```cpp
auto simplified = planning::utils::SimplifyPath(path, epsilon);
auto smoother = std::make_shared<planning::utils::SimpleSmoother>(...);
smoother->Smooth(path, std::chrono::milliseconds(1000));
```

> `path_simplify_epsilon > 0` 可能使绕障路径坍缩为直线，默认保持 `0.0`。

### 4.6 自定义插件

1. 继承 `common::GlobalPlanner`，实现 `CreatePlan()`
2. `AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(MyPlanner, GlobalPlanner)`
3. 编写 XML 插件描述，在 `planner.lua` 中注册

详见 [05_architecture.md §5.9](05_architecture.md)。

### 4.7 规划器选型

| 场景 | 推荐 | 关键配置 |
|------|------|----------|
| 通用室内 | `navfn_planner` | 默认 |
| 确定性调试 | `dijkstra_planner` | — |
| 短路径/开阔 | `theta_star_planner` | `how_many_corners=8` |
| 窄通道 | NavFn 或 Theta* | `how_many_corners=4` |
| 大地图 | `navfn_planner` | `use_astar=true` |
| 未知区域 | 任意 | `allow_unknown=true` |

更多场景见 [09_survey.md §9.14](09_survey.md)。

### 4.8 故障排查

| 异常 | 常见原因 | 处理 |
|------|----------|------|
| `NoValidPathCouldBeFound` | 起终点不连通 | 检查 costmap，增大 `tolerance` |
| `StartOccupied` / `GoalOccupied` | 起终点在障碍上 | 调整位姿或膨胀参数 |
| `InvalidPlanner` | 插件 id 错误 | 检查 `planner_plugins` |
| `PlannerTimedOut` | costmap 未更新 | 增大 `costmap_update_timeout` |
| `PlannerTFError` | 坐标系错误 | 检查 TF 与 `frame_id` |

**检查清单**：`costmap->isCurrent()` · `worldToMap` 边界 · 障碍/膨胀层 · 启动日志 `PlannerServer has N planners`。

### 4.9 性能建议

| 建议 | 说明 |
|------|------|
| 地图 ≤ 2048² | 控制全局 costmap 规模 |
| 更新 5 Hz | `update_frequency` 通常足够 |
| 按需重规划 | 依赖 `IsPathValid`，避免全图重算 |
| 禁用 DP 简化 | `path_simplify_epsilon = 0` |
| 大地图用 A* | `use_astar = true` |
