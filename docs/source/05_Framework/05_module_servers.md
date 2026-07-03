# 5. 模块 Server

各子系统以 **Server** 类作为对外服务入口，由 `system::Autonomy` 统一构造与持有。

### 5.1 Server 一览

| Server | 类 | 构造时机 | 主要职责 |
|--------|-----|----------|----------|
| 地图 | `map::MapServer` | `Start()` | 加载 PGM/YAML，发布 `OccupancyGrid` |
| 规划 | `planning::PlannerServer` | `Start()` | 全局规划、costmap、路径校验 |
| 控制 | `control::ControllerServer` | `Start()` | FollowPath、里程计、cmd_vel |
| 变换 | `transform::TransformServer` | `Start()` | 静态外参 → `tf_buffer` |
| 编排 | `navigator`（BT） | `Configure()` | 行为树导航（待完整恢复） |

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
- 与 Planner 共享 costmap：`SetSharedCostmap()`
- 关键 API：`Start()`、`GetLatestOdometry()`、`TickFollowPath()`（演进中）

详见 [09 Control](../09_Control/02_architecture.md)。

### 5.5 TransformServer + tf_buffer

- `TransformServer` 从配置加载静态外参
- `transform::Buffer::Instance()` 单例，供规划/控制/Navigator 查询 TF
- Navigator BT 节点 `TransformAvailable` 依赖此缓冲

### 5.6 SensorCollator

- 聚合激光等传感器数据
- 回调注入 `ControllerServer`、costmap `feedLaserScan`

### 5.7 Autonomy 对外 API

```cpp
bool NavigateToPose(goal, cancel_checker, keep_alive, timeout_sec);
bool NavigateThroughPoses(goals, ...);
void ReplanToGoal(goal);
map::MapServer* GetMapServer();
planning::PlannerServer* GetPlanner();  // via planner_
control::ControllerServer* GetController();
```

### 5.8 Server 与 Autolink

各 Server **内部**创建 autolink Node（如 `planner_server`），对外暴露 C++ API 或 Action，应用层通常不直接操作 Node。

```
应用 → Autonomy::NavigateToPose
         → navigator BT → Action Client
              → PlannerServer Action / GetPlan()
```

### 5.9 相关文档

- [§3 框架架构](03_architecture.md)
- [04 Running · Autonomy 进程](../04_Running/03_autonomy_process.md)
