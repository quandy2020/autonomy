(control-architecture)=
# 1. Control 模块架构

上手 [§0](00_guide.md)。

### 2.1 位置

```
Planning.Path → ControllerServer → ControllerInterface
├── Goal / Progress Checker
├── VelocitySmoother（可选）
└── Costmap（local 或共享 global）
↓ cmd_vel
```

入口：`AUTONOMY_CONTROLLER` → `ControllerServer`。

### 2.2 分层与状态

| 层 | 状态 |
|----|------|
| ControllerServer 骨架 | — |
| FollowPath / ComputeControl | stub |
| 控制器插件 | 接口定、未加载 |
| Goal / Progress Checker | （Lua 接线 ） |
| VelocitySmoother | （节点接线 ） |

### 2.3 FollowPath 循环（设计意图）

`SetPlan` → 周期：取位姿 → ProgressCheck → 未到则 `ComputeVelocityCommands` → GoalCheck → 可选 Smoother → 发 `cmd_vel`；无进度则失败。

频率：`controller_frequency`。失败容忍：`failure_tolerance`。

### 2.4 扩展

实现 `ControllerInterface`，注册插件，在 `controller_plugins` 启用。Checker 实现 `GoalChecker` / `ProgressChecker`。
