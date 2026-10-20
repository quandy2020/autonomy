(control-architecture)=
# 1. Control 模块架构

## 背景

Control 负责 **跟全局 Path 发 `cmd_vel`**：`ControllerServer` 从配置加载控制器插件，在 FollowPath 循环里调用 `ComputeVelocityCommands`，并用 Goal / Progress Checker 判到达与卡住。

## 思路

FollowPath 已在 `ControllerServer` 闭环实现：控制器插件产出 `cmd_vel`，默认 Goal/Progress 为 Simple 系列；VelocitySmoother 库存在但未接在出口。读数据流图后对照 `controller_server.*` 与 conf 中的插件 id。


## 步骤

```text
Planning.Path → ControllerServer
                  ├── 控制器插件（RPP / Graceful / MPPI / …）
                  ├── GoalChecker（默认 SimpleGoal）
                  ├── ProgressChecker（默认 SimpleProgress）
                  ├── Costmap2DWrapper（共享或局部）
                  └── OdomSmoother（平滑里程计读数，不是 cmd_vel 平滑）
                → cmd_vel
```

配置入口：`AUTONOMY_CONTROLLER` / `ControllerOptions`；实现：`autonomy/control/controller_server.*`。

### 1. FollowPath 循环（已实现）

1. `ComputeControl()`：校验 path、解析 controller / checker id，`SetPlan`，置 `follow_path_active_`。
2. 周期 `ComputeAndPublishVelocity()`（频率 `controller_frequency`）：
   - TF 取位姿；里程计填当前速度；
   - `progress_checker->Check` 失败 → `FailedToMakeProgress`；
   - `controller->ComputeVelocityCommands` → `cmd_vel`；
   - `goal_checker->IsGoalReached` → 结束并发零速。

Policy 控制器推理较慢，实现里对 costmap 加锁策略与其它插件不同（避免长时间持锁）。

### 2. 插件加载

- **控制器**：`controller_plugins` 列表，经 PluginManager 注册名加载；空列表会 `AFATAL`。
- **Checker**：`LoadPlugins()` 里 **写死** 只构造 `SimpleGoalChecker` 与 `SimpleProgressChecker`，容差来自 `checker_options`。`PositionGoalChecker`、`StoppedGoalChecker`、`PoseProgressChecker` 有源码，**尚未** 挂进 Server 注册表；要用需改 `controller_server.cpp` 或后续插件化接线。
- **VelocitySmoother**：`utils/velocity_smoother.*` 库存在，**未** 在 FollowPath 出口默认调用；见 [03](03_velocity_smoother.md)。

### 3. 扩展新控制器

实现 `ControllerInterface` → `ComputeVelocityCommands` / `SetPlan` / `Reset` → `AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN` → 在 conf 的 `controller_plugins` 里写 id。

Checker 扩展：实现 `GoalChecker` / `ProgressChecker`，并在 Server 里像 Simple 一样 `make_shared` + 插入 map（当前需改 C++）。

### 4. 与 Planning / Map

Planning 只给几何 Path；局部避障靠控制器内 costmap 查询（RPP regulated、MPPI critic 等）。`SetSharedCostmap` 可与 Planner 共用同一份 `Costmap2DWrapper`。

上手与选型：[00 指南](00_guide.md)、[04 算法总览](04_controller_algorithms.md)、[05 综述](05_survey.md)。
