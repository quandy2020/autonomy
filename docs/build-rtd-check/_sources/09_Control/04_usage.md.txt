(control-usage)=
# 4. 使用指南

### 4.1 配置

**入口文件**：`config/control/controller.lua` → `config/autonomy.lua`

```lua
AUTONOMY = {
    control = AUTONOMY_CONTROLLER,
}
```

**关键字段**：

| 字段 | 说明 | 默认建议 |
|------|------|----------|
| `controller_frequency` | 控制循环频率 (Hz) | `20.0`（与 MPPI `model_dt=0.05` 匹配） |
| `failure_tolerance` | 连续无效命令容忍时间 (s) | `30.0` |
| `publish_zero_velocity` | 退出时是否发零速 | `false` |
| `controller_plugins` | 启用的控制器插件 | 待实现后启用 |
| `goal_checker.*` | 目标到达容差 | 见下表 |
| `progress_checker.*` | 进度检测参数 | 见下表 |
| `costmap.enabled` | 是否启用独立局部 costmap | 附加模式设 `false` |

**Goal Checker 配置**（`controller.lua` 顶层）：

| 参数 | 含义 | 默认 |
|------|------|------|
| `xy_goal_tolerance` | XY 位置容差 (m) | `AUTONOMY_COMMON.goal_reached_tolerance` |
| `yaw_goal_tolerance` | 航向容差 (rad) | `0.35` |
| `stateful` | XY 达标后不再重检 XY | `true` |

**Progress Checker 配置**：

| 参数 | 含义 | 默认 |
|------|------|------|
| `required_movement_radius` | 判定"有进度"的最小位移 (m) | `0.5` |
| `movement_time_allowance` | 允许无进度的时间 (s) | `10.0`（proto 已定义，C++ 待实现） |

> **注意**：Lua 中 costmap 键名为 `costmap`，而 `LoadOptions()` 读取 `costmap_2d_options`。当前本地 costmap 不会从 Lua 自动加载；附加模式下通过 `SetSharedCostmap()` 共享 planner 全局地图。

C++ 加载：

```cpp
auto options = autonomy::control::LoadOptions(dict.get());
auto server = std::make_shared<autonomy::control::ControllerServer>(options);
```

### 4.2 ControllerServer API

| API | 用途 | 状态 |
|-----|------|------|
| `Start()` / `Shutdown()` | 启停 costmap 线程 | ✅ |
| `SetSharedCostmap(costmap)` | 注入共享 costmap | ✅ |
| `UpdateOdometry(odom)` | 更新里程计 | ✅ |
| `GetLatestOdometry(odom)` | 读取最新里程计 | ✅ |
| `GetRobotPose(pose)` | 获取机器人位姿 | ✅ |
| `ComputeControl()` | FollowPath 主循环 | ⏳ stub |
| `FindControllerId(name, id)` | 解析控制器插件 id | ✅ |
| `FindGoalCheckerId(name, id)` | 解析 goal checker id | ✅ |
| `FindProgressCheckerId(name, id)` | 解析 progress checker id | ✅ |

### 4.3 ControllerInterface 插件 API

继承 `common::ControllerInterface` 并实现：

| 方法 | 说明 |
|------|------|
| `ComputeVelocityCommands(pose, vel, cmd_vel, goal_checker, msg)` | 核心：计算速度命令，返回 `ControllerResultCode` |
| `SetPlan(path)` | 接收/更新全局路径 |
| `IsGoalReached(dist_tol, angle_tol)` | 控制器内部终点判定 |
| `SetSpeedLimit(speed, percentage)` | 动态限速 |
| `Reset()` | 导航结束重置状态 |

**返回值** `ControllerResultCode`：

| 码 | 含义 |
|----|------|
| `0` | SUCCESS |
| `102` | NO_VALID_CMD |
| `104` | COLLISION |
| `106` | ROBOT_STUCK |
| `111` | INVALID_PATH |
| `112` | TF_ERROR |

### 4.4 系统集成

```text
// system::Autonomy 构造流程
controller_ = std::make_shared<control::ControllerServer>(options_.controller_options());
controller_->SetSharedCostmap(planner_->GetCostmapWrapper());
controller_->Start();

// 传感器回调
sensor_consumer->OnOdometry() → controller_->UpdateOdometry(odom);
```

```
用户目标 → Navigator BT → FollowPath → ControllerServer
                              ↑              ↓
                        Planning.Path    cmd_vel → 底盘
                              ↑
                        global costmap（共享）
```

### 4.5 通信接口（设计意图）

| 类型 | 名称 | 说明 |
|------|------|------|
| 节点 | `controller_server` | autolink 节点 |
| 话题 | `cmd_vel` | 速度输出（待接线） |
| 话题 | `odom` | 里程计输入 |
| Action | `FollowPath` | 行为树 FollowPath 节点 |
| 代价地图 | `/local_costmap` 或共享 `/global_costmap` | 碰撞检测 |

### 4.6 自定义控制器插件

1. 继承 `common::ControllerInterface`
2. 实现 `ComputeVelocityCommands()`、`SetPlan()` 等虚函数
3. `AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(MyController, ControllerInterface)`
4. 在 `controller.lua` 的 `controller_plugins` 中注册 `"my_controller:MyController"`

参考 `PlannerServer` 插件加载模式（`ParsePluginSpecs` → `RegisterInProcessClass` → `CreateInstance`），详见 [§5.9 插件加载](05_architecture.md#59-插件加载设计)。

### 4.7 控制器选型

| 场景 | 推荐 | 关键配置 |
|------|------|----------|
| 室内差速、窄通道 | Graceful Controller | `use_collision_detection=true` |
| 动态避障、复杂环境 | MPPI | `batch_size=2000`, CostCritic |
| 简单跟踪、低算力 | Pure Pursuit / RPP | 小 lookahead |
| 高精度停车 | StoppedGoalChecker | `trans/rot_stopped_velocity` |
| 只关心位置 | PositionGoalChecker | 忽略航向 |

更多场景见 [09_survey.md §9.14](09_survey.md#914-工程选型矩阵)。

### 4.8 故障排查

| 异常 / 现象 | 常见原因 | 处理 |
|-------------|----------|------|
| `FailedToMakeProgress` | 机器人卡住、路径被挡 | 增大 `required_movement_radius` 或检查 costmap |
| `NoValidControl` | 控制器无法生成有效命令 | 检查路径有效性、costmap 更新 |
| `ControllerTFError` | TF 不可用 | 检查 `global_frame` / `base_link` |
| `InvalidPath` | 空路径或路径点过少 | 确认 planning 输出 |
| 控制命令始终为零 | `ComputeControl` 未实现 | 当前已知限制，待开发 |
| Checker 参数不生效 | Lua 未接线到 C++ | 使用 `SetTolerances()` 手动设置 |
| 局部 costmap 未加载 | Lua 键名 `costmap` vs `costmap_2d_options` | 使用 `SetSharedCostmap()` |

**检查清单**：`GetRobotPose()` 返回 true · odom 正常更新 · costmap `isCurrent()` · 启动日志插件数量 · `controller_frequency` 与 `model_dt` 一致。

### 4.9 性能建议

| 建议 | 说明 |
|------|------|
| 控制频率 20 Hz | 与 MPPI `model_dt=0.05` 对齐 |
| 共享全局 costmap | 附加模式减少重复计算 |
| 启用 VelocitySmoother | 保护硬件、平滑加速度 |
| MPPI batch_size | 2000 适合桌面 CPU；嵌入式可降至 500–1000 |
| failure_tolerance | 短时 TF 抖动设 5–30 s |
