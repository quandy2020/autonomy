# 0. Control 指南

## 背景

有 Path 却没有 `cmd_vel`、到点不停、`FailedToMakeProgress`——局部控制环和 checker 配置是导航闭环里最容易误判的一层。本章讲 **吃全局 Path、吐 `cmd_vel`**，对应 Nav2 controller；架构见 [01](01_architecture.md)。

## 思路

```text
Path → ControllerServer → cmd_vel
         ↑ Goal/Progress Checker
         ↑（库有 VelocitySmoother，控制环尚未默认挂上）
```

- **`ControllerServer` 默认只挂 SimpleGoal / SimpleProgress**；其它 checker 类在仓库里，但还没进注册表。  
- **`controller_plugins` 字符串** 必须和实现注册名一致。  
- costmap / TF / 里程计缺一会没输出；跟人不走 OccupancyGrid，Follow 出 GridMap，需 MPPI 侧 Grid critic（[Perception · Follow](../10_Perception/11_follow.md)）。

已有控制器（均有源码）：

| 插件 | 适合 | 文档 |
|------|------|------|
| RPP / Graceful | 室内默认、算力紧 | [03 RPP](controller/03_rpp_controller.md) · [01 Graceful](controller/01_graceful_controller.md) |
| MPPI | 动态障碍、Follow GridMap critic | [02 MPPI](controller/02_mppi_controller.md) |
| NMPC | 显式时域约束 | [04 NMPC](controller/04_nmpc_controller.md) |
| NeuPAN | ONNX 距离场 | [05 NeuPAN](controller/05_neupan_controller.md) |
| SANDO | 走廊 + MIQP | [06 SANDO](controller/06_sando_controller.md) |
| Policy | 自训策略图 | [07 Policy](controller/07_policy_controller.md) |

Checker：[02](02_checkers.md)。速度平滑库：[03](03_velocity_smoother.md)（**未默认接入控制环**）。综述：[05](05_survey.md)。


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 架构](01_architecture.md) | ControllerServer 与主循环 |
| [02 Checker](02_checkers.md) | Goal / Progress |
| [03 VelocitySmoother](03_velocity_smoother.md) | 平滑库（未默认接线） |
| [04 算法总览](04_controller_algorithms.md) | 插件怎么选 |
| [05 综述](05_survey.md) | 与 Nav2 对照 |
| `controller/*.md` | 各控制器参数 |

**先让车跟着路径走** → RPP + [02](02_checkers.md)。  
**躲行人 / 动态障** → MPPI 或 NeuPAN。  
**抖动** → 容差、膨胀、Progress 时限；平滑见 [03](03_velocity_smoother.md)。

### 2. 最短配置

```lua
AUTONOMY_CONTROLLER = {
    controller_frequency = 20.0,
    goal_checker = { xy_goal_tolerance = 0.25, yaw_goal_tolerance = 0.35, stateful = true },
    progress_checker = { required_movement_radius = 0.5, movement_time_allowance = 10.0 },
}
```

```cpp
auto server = std::make_shared<ControllerServer>(LoadOptions(...));
server->Start();
server->SetSharedCostmap(planner->GetCostmapWrapper());
```

### 3. 常见坑

| 现象 | 先查 |
|------|------|
| 到不了点不停 | xy/yaw 容差；StoppedGoal 需自行接线 |
| `FailedToMakeProgress` | 障碍、膨胀、Progress 半径/时限 |
| 无 `cmd_vel` | 插件是否加载；主循环；发布侧 |
| 跟人不跟踪 | Follow 话题；MPPI Grid critic 是否开 |
