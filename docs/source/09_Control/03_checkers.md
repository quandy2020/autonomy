# 3. 目标与进度检查器

Checker 是 Control 模块的**判定子系统**，独立于局部控制器运行。Goal Checker 判定是否到达目标；Progress Checker 判定机器人是否在向目标前进（防卡住）。

> 各 Checker 实现专题见 `checker/15_*`–`19_*`（文件名前缀；与 `controller/15_mpc` 不同目录）。文档地图见 [00_guide §0.1](00_guide.md#01-阅读路径与文档地图)。

## 3.1 接口设计

### GoalChecker

```cpp
virtual bool IsGoalReached(
    const Pose& query_pose,
    const Pose& goal_pose,
    const Twist& velocity) = 0;

virtual bool GetTolerances(Pose& pose_tol, Twist& vel_tol) = 0;
virtual void Reset() = 0;
```

### ProgressChecker

```cpp
virtual bool Check(PoseStamped& current_pose) = 0;
virtual void Reset() = 0;
```

**语义约定**：

- GoalChecker：`true` = 已到达
- ProgressChecker：`true` = **有进度**（机器人移动足够）；`false` = 无进度 → 触发 `FailedToMakeProgress`

## 3.2 SimpleGoalChecker

六段式专题：**[§15 SimpleGoalChecker](checker/15_simple_goal_checker.md)**（背景 → 问题 → 误差模型 → 到达条件 → 算法 1–2）。

**文件**：`checker/simple_goal_checker.*` · 默认 XY 0.25 m、yaw 0.25 rad、`stateful=true`。

## 3.3 PositionGoalChecker

**[§16 PositionGoalChecker](checker/16_position_goal_checker.md)** · 仅 XY，忽略航向 · `checker/position_goal_checker.*`

## 3.4 StoppedGoalChecker

**[§17 StoppedGoalChecker](checker/17_stopped_goal_checker.md)** · Simple + 速度停稳 · `checker/stopped_goal_checker.*`

## 3.5 SimpleProgressChecker

**[§18 SimpleProgressChecker](checker/18_simple_progress_checker.md)** · XY 位移进度 · `movement_time_allowance` C++ 待实现

## 3.6 PoseProgressChecker

**[§19 PoseProgressChecker](checker/19_pose_progress_checker.md)** · XY 或航向变化算进度 · `checker/pose_progress_checker.*`

## 3.7 Checker 对比

| Checker | 检测维度 | Stateful | 速度检测 | 专题 | 典型用途 |
|---------|----------|----------|----------|------|----------|
| SimpleGoalChecker | XY + Yaw | ✅ | ❌ | [§15](checker/15_simple_goal_checker.md) | 通用导航 |
| PositionGoalChecker | XY only | ✅ | ❌ | [§16](checker/16_position_goal_checker.md) | 只关心位置 |
| StoppedGoalChecker | XY + Yaw + Vel | ✅ | ✅ | [§17](checker/17_stopped_goal_checker.md) | 精确停车 |
| SimpleProgressChecker | XY 位移 | — | ❌ | [§18](checker/18_simple_progress_checker.md) | 防卡住 |
| PoseProgressChecker | XY + Yaw 变化 | — | ❌ | [§19](checker/19_pose_progress_checker.md) | 含旋转进度 |

## 3.8 配置与集成

**Lua 配置**（`controller.lua`）：

```lua
goal_checker = {
    xy_goal_tolerance = 0.25,
    yaw_goal_tolerance = 0.35,
    stateful = true,
},
progress_checker = {
    required_movement_radius = 0.5,
    movement_time_allowance = 10.0,
},
```

**与控制器配对**（详见 [§5.8.1](05_controller_algorithms.md#581-控制器与-checker-配对)、[§0.9.1](00_guide.md#091-局部轨迹与时空联合选型)）：

| 控制器族 | 推荐 Goal Checker | 推荐 Progress |
|----------|-------------------|---------------|
| RPP / Graceful | Simple | Simple；先转后走 → Pose |
| MPPI / DWB | Simple | Simple |
| TEB / NMPC | Stopped（对接）/ Simple | Pose（多原地转） |
| 只到点任务 | Position | Simple |

**Proto 定义**（`checker_options.proto`）：

```protobuf
message GoalCheckerOptions {
    bool stateful = 1;
    string plugin = 2;
    double xy_goal_tolerance = 3;
    double yaw_goal_tolerance = 4;
}
message ProgressCheckerOptions {
    string plugin = 1;
    double required_movement_radius = 2;
    double movement_time_allowance = 3;
}
```

> **当前限制**：C++ `Initialize()` 内使用硬编码默认值，标注 `TODO: Load parameters from configuration`。临时方案：构造后调用 `SetTolerances()` / `SetXYGoalTolerance()`。完整 Lua→Proto→C++ 接线状态见 [§3.13](#313-lua-proto-c-接线状态)。

## 3.9 在 FollowPath 中的调用时序

```
每个控制周期 (@ controller_frequency):
  1. progress_checker->Check(current_pose)
     └─ false → 累计无进度时间 → 超时 throw FailedToMakeProgress

  2. controller->ComputeVelocityCommands(..., goal_checker, ...)
     └─ goal_checker 传入控制器，用于控制器内部终点判定

  3. IsGoalReached()
     └─ goal_checker->IsGoalReached(pose, end_pose, velocity)
        └─ true → 退出循环
```

## 3.10 调参建议

| 场景 | xy_tolerance | yaw_tolerance | progress radius |
|------|-------------|---------------|-----------------|
| 室内通用 | 0.25 m | 0.35 rad | 0.5 m |
| 精密对接 | 0.05 m | 0.1 rad | 0.1 m |
| 开阔场地 | 0.5 m | 0.5 rad | 1.0 m |
| 原地转向任务 | 0.25 m | 0.35 rad | 0.5 m + PoseProgressChecker |

**注意**：`xy_goal_tolerance` 应与 Navigator BT 中 `GoalReached` 节点的容差保持一致（`AUTONOMY_COMMON.goal_reached_tolerance`）。

## 3.13 Lua → Proto → C++ 接线状态

Checker **算法已实现**，但配置管线与 `ControllerServer` 插件加载尚未闭环。下表描述当前仓库真实状态（以 `control_options.cpp`、`checker_options.proto` 为准）。

### 3.13.1 配置管线

```
config/control/controller.lua
    │  AUTONOMY_CONTROLLER.goal_checker / progress_checker
    ▼
control::LoadOptions()          ← 仅读 controller_frequency、plugins、costmap_2d_options 等
    │  ❌ 未读 goal_checker / progress_checker
    ▼
proto::ControllerOptions        ← 无 CheckerOptions 字段（checker 在独立 checker_options.proto）
    ▼
ControllerServer                ← goal_checkers_ / progress_checkers_ 容器已声明；插件实例化 ⏳
    ▼
SimpleGoalChecker::Initialize() ← TODO: Load parameters；当前硬编码 + SetTolerances() 临时方案
```

**VelocitySmoother** 同理：`VelocitySmootherOptions` 在 `smoother_options.proto`，`controller.lua` **尚无**顶层 `velocity_smoother` 段；见 [§4.12](04_velocity_smoother.md#412-配置接线状态)。

### 3.13.2 Lua 字段 → Proto 映射（已定义 / 待接线）

**顶层** `controller.lua`（与 Navigator `AUTONOMY_COMMON.goal_reached_tolerance` 对齐）：

| Lua 键（`goal_checker`） | `GoalCheckerOptions` 字段 | C++ 消费 | 状态 |
|--------------------------|---------------------------|----------|------|
| `plugin` | `plugin` | 插件类名 / `FindGoalCheckerId` | Lua 未写；proto 已定义 |
| `xy_goal_tolerance` | `xy_goal_tolerance` | 各 Checker `Initialize` | Lua ✅ · Load ⏳ |
| `yaw_goal_tolerance` | `yaw_goal_tolerance` | Simple / Stopped | Lua ✅ · Load ⏳ |
| `stateful` | `stateful` | Simple / Position | Lua ✅ · Load ⏳ |
| — | `path_length_tolerance` | Nav2 扩展 | proto 预留 |

| Lua 键（`progress_checker`） | `ProgressCheckerOptions` 字段 | C++ 消费 | 状态 |
|------------------------------|--------------------------------|----------|------|
| `plugin` | `plugin` | 插件类名 | Lua 未写 · proto 已定义 |
| `required_movement_radius` | `required_movement_radius` | Simple / Pose `radius_` | Lua ✅ · Load ⏳ |
| `movement_time_allowance` | `movement_time_allowance` | Nav2 时间窗口 | Lua ✅ · **C++ 未实现** |

**StoppedGoalChecker 扩展**（Nav2 插件参数，Autonomy proto **尚未**定义）：

| Nav2 参数 | C++ 成员 | 默认 |
|-----------|----------|------|
| `trans_stopped_velocity` | `trans_stopped_velocity_` | 0.25 m/s |
| `rot_stopped_velocity` | `rot_stopped_velocity_` | 0.25 rad/s |

**PoseProgressChecker 扩展**：

| 参数 | C++ 成员 | 默认 |
|------|----------|------|
| `required_movement_angle` | `required_movement_angle_` | 0.5 rad |

### 3.13.3 与 `mppi_controller` 嵌套块的关系

`controller.lua` 内 `mppi_controller.goal_checker` / `progress_checker` 沿用 **Nav2 字符串插件名**（如 `nav2_controller::SimpleGoalChecker`），供未来 MPPI 插件自读参数；**不等于** Autonomy `ControllerServer` 顶层 Checker 管线。FollowPath 应以 **顶层** `goal_checker` / `progress_checker` 为单一真相源（待 `LoadOptions` 接入）。

### 3.13.4 工程接线清单（P1）

```
□ ControllerOptions 增加 checker_options 字段（或 LoadOptions 合并读取 CheckerOptions）
□ control_options.cpp 解析 goal_checker / progress_checker 字典
□ ControllerServer 构造：按 plugin 字段实例化 Checker + Initialize(options)
□ SimpleProgressChecker：实现 movement_time_allowance 计时
□ GoalCheckerOptions 扩展 stopped / pose 专有字段（或插件级子字典）
□ 单元测试：Lua 容差 → IsGoalReached 边界
```

**临时集成**（测试 / 单测）：

```cpp
auto gc = std::make_shared<checker::SimpleGoalChecker>();
gc->Initialize("goal_checker", nullptr);
gc->SetTolerances(
    lua_xy, lua_yaw, lua_stateful);  // 手动对齐 controller.lua
```

插件选型与控制器配对见 [§5.8.1](05_controller_algorithms.md#581-控制器与-checker-配对)。
