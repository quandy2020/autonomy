# 5. 模块 Server

各子系统以 **Server** 类作为对外服务入口，由对应模块进程（`*_main`）构造并运行；共享 conf 可用 `system::CreateOptions` 加载 `AutonomyOptions` 子字段。

### 5.1 Server 一览

| Server | 类 | 典型进程 | 主要职责 |
|--------|-----|----------|----------|
| 地图 | `map::MapServer` | planning / map 相关 | 加载 PGM/YAML，发布 `OccupancyGrid` |
| 规划 | `planning::PlannerServer` | `autonomy.planning` | 全局规划、costmap、路径校验 |
| 控制 | `control::ControllerServer` | `autonomy.control` | FollowPath、里程计、cmd_vel |
| 变换 | `transform::TransformServer` | transform / 各进程内 | 静态外参 → `tf_buffer` |
| 编排 | `task::TaskServer` | `autonomy.task` | 行为树任务 / 导航编排 |

进程内 `system::Autonomy` **已移除**；用 `autolink_launch autonomy.launch` 拉起上述进程。

### 5.2 MapServer

- 输入：`MapOptions`（地图路径、帧名）
- 输出：`OccupancyGrid` 回调 → `PlannerServer` costmap 静态层
- 关键 API：`Start()`、`GetStaticMapShared()`、`SetMapPublishCallback()`

### 5.3 PlannerServer

- 输入：`PlannerOptions`（规划器插件、costmap 配置）
- 输出：`planning_msgs::Path`
- 关键 API：
  - `GetPlan(start, goal, planner_id, cancel_checker)`
  - `IsPathValid(path)`
  - `GetCostmapWrapper()`

详见 [08 Planning](../08_Planning/01_architecture.md)。

### 5.4 ControllerServer

- 输入：`ControllerOptions`
- 输出：`geometry_msgs::TwistStamped`
- 与 Planner 共享 costmap：`SetSharedCostmap()`（进程内）或经 IPC
- 关键 API：`Start()`、`GetLatestOdometry()`、`TickFollowPath()`（演进中）

详见 [09 Control](../09_Control/02_architecture.md)。

### 5.5 TransformServer + tf_buffer

- `TransformServer` 从配置加载静态外参
- `transform::Buffer` 供规划/控制/Task 查询 TF
- BT 节点 `TransformAvailable` 依赖此缓冲

### 5.6 SensorCollator

- 聚合激光等传感器数据
- 回调注入 `ControllerServer`、costmap `feedLaserScan`

### 5.7 任务发令

应用层通过 **Bridge / autolink Action** 向 `autonomy.task` 发令，而不是进程内 `Autonomy::NavigateToPose`。

```
Bridge / Action Client → TaskServer
         → BT / 导航任务 → PlannerServer / ControllerServer（跨进程）
```

### 5.8 Server 与 Autolink

各 Server **内部**创建 autolink Node，对外暴露 C++ API 或 Action；多进程部署时跨进程经 Channel / Service / Action。

### 5.9 相关文档

- [§3 框架架构](03_architecture.md)
- [04 Running · 多进程栈](../04_Running/03_autonomy_process.md)
