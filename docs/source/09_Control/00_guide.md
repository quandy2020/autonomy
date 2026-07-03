(control-guide)=
# 0. Control 运动控制指南

`autonomy/control` 是 Autonomy 的**局部运动控制**子系统，对齐 ROS 2 Navigation2 `nav2_controller`。接收全局路径，以固定频率计算并发布 `cmd_vel`，直至目标到达。本文档为模块总入口（**§0**）：概览、形式化、快速开始、配置与排错。

**编号约定**：

| 范围 | 含义 |
|------|------|
| **§0–§6** | 模块主干（§0 本页；§2 架构；§3–§5 组件；§6 综述） |
| **`controller/10_*`–`15_*`** | 局部控制器算法专题（对应 §5.2–§5.7；文中简称 **§10–§15**） |
| **`checker/15_*`–`19_*`** | Goal / Progress Checker 专题（对应 §3.2–§3.6） |
| **`smoother/20_*`** | VelocitySmoother 实现专题（对应 §4） |

专题编号取自**文件名前缀**；`controller/15_mpc` 与 `checker/15_simple_goal` 分属不同子目录，互不冲突。

## 0.1 阅读路径与文档地图

| 角色 | 建议顺序 |
|------|----------|
| 新手 | 本章 §0.2–§0.3 → §0.10 快速开始 → [§2 架构](02_architecture.md) |
| 算法研发 | [§6 综述](06_survey.md) → 本章 §0.6–§0.9 → [§5 局部控制器](05_controller_algorithms.md) → [§10–§15 控制器专题](#control-controller-topics) |
| 集成调试 | §0.10–§0.17 → [§3 检查器](03_checkers.md) → [§0.19 排错](#019-故障排查) |

| § | 文件 | 内容 |
|---|------|------|
| 0 | [00_guide.md](00_guide.md) | 本指南（概览、形式化、上手、排错） |
| 2 | [02_architecture.md](02_architecture.md) | 模块架构设计 |
| 3 | [checker/index.rst](checker/index.rst) → [03_checkers.md](03_checkers.md) | 目标与进度检查器 |
| 4 | [smoother/index.rst](smoother/index.rst) → [04_velocity_smoother.md](04_velocity_smoother.md) | 速度平滑器 |
| 5 | [controller/index.rst](controller/index.rst) → [05_controller_algorithms.md](05_controller_algorithms.md) | 局部控制器 |
| 6 | [06_survey.md](06_survey.md) | 运动控制算法综述 |

(control-controller-topics)=
### §10–§15 局部控制器专题（`controller/10_*`–`15_*`，§5 详读）

| 文件前缀 | 文件 | 对应 §5 | 算法 |
|------|------|---------|------|
| 10 | [controller/10_graceful_controller.md](controller/10_graceful_controller.md) | §5.2 | Graceful Controller |
| 11 | [controller/11_mppi_controller.md](controller/11_mppi_controller.md) | §5.3 | MPPI Controller |
| 12 | [controller/12_rpp_controller.md](controller/12_rpp_controller.md) | §5.4 | Regulated Pure Pursuit |
| 13 | [controller/13_dwb_controller.md](controller/13_dwb_controller.md) | §5.5 | DWB Controller |
| 14 | [controller/14_teb_controller.md](controller/14_teb_controller.md) | §5.6 | TEB Controller |
| 15 | [controller/15_mpc_controller.md](controller/15_mpc_controller.md) | §5.7 | MPC Controller |

### Checker 专题（`checker/15_*`–`19_*`，§3 详读）

| 文件前缀 | 文件 | 对应 §3 | 组件 |
|------|------|---------|------|
| 15 | [checker/15_simple_goal_checker.md](checker/15_simple_goal_checker.md) | §3.2 | SimpleGoalChecker |
| 16 | [checker/16_position_goal_checker.md](checker/16_position_goal_checker.md) | §3.3 | PositionGoalChecker |
| 17 | [checker/17_stopped_goal_checker.md](checker/17_stopped_goal_checker.md) | §3.4 | StoppedGoalChecker |
| 18 | [checker/18_simple_progress_checker.md](checker/18_simple_progress_checker.md) | §3.5 | SimpleProgressChecker |
| 19 | [checker/19_pose_progress_checker.md](checker/19_pose_progress_checker.md) | §3.6 | PoseProgressChecker |

### Smoother 专题（`smoother/20_*`，§4 详读）

| 文件前缀 | 文件 | 内容 |
|------|------|------|
| 20 | [smoother/20_velocity_smoother_impl.md](smoother/20_velocity_smoother_impl.md) | VelocitySmoother 源码级实现 |

## 0.2 模块定位

| 维度 | 说明 |
|------|------|
| 控制层级 | 局部轨迹跟踪（Local Controller / Trajectory Tracker） |
| 输入 | 全局路径 `planning_msgs::Path`、当前位姿、当前速度、局部/全局代价地图 |
| 输出 | `geometry_msgs::TwistStamped`（`cmd_vel`） |
| 上游 | `planning`（路径）、`localization`（位姿）、`map`（costmap） |
| 下游 | 底盘驱动、仿真器 |
| 对标 | nav2_controller、nav2_velocity_smoother |

```
Planning.Path ──→ ControllerServer ──→ cmd_vel ──→ 底盘
       ↑                ↑
  global costmap    local costmap（可选）
       ↑                ↑
   map 模块         传感器 / 障碍层
```

## 0.3 核心能力

| 能力 | 状态 | 说明 |
|------|------|------|
| `ControllerServer` 服务骨架 | ✅ 部分 | 构造、位姿获取、里程计接入已实现 |
| FollowPath 控制循环 | ⏳ 待完成 | `ComputeControl()` 等核心方法为 stub |
| 插件化控制器 | ⏳ 待完成 | `ControllerInterface` 已定义，加载逻辑待接 Autolink |
| Goal Checker | ✅ 已实现 | Simple / Position / Stopped 三种 |
| Progress Checker | ✅ 已实现 | Simple / Pose 两种 |
| VelocitySmoother | ✅ 算法已实现 | 加速度约束平滑，节点接线待完成 |
| 局部 costmap | ✅ 可选 | 可独立启用或与 planner 共享全局 costmap |
| 配置管线 | ✅ 部分 | Lua → `ControllerOptions` protobuf |

> **当前阶段**：Checker、Smoother、几何工具已就绪；`ControllerServer` 主循环与控制器插件尚待实现。详见 [§2.2 实现状态](02_architecture.md#22-分层与实现状态)。

## 0.4 源码结构

```
autonomy/control/
├── controller_server.*           # FollowPath 服务入口
├── control_options.*             # Lua → ControllerOptions
├── common/
│   ├── controller_interface.hpp  # 局部控制器插件接口
│   ├── goal_checker_interface.hpp
│   ├── progress_checker_interface.hpp
│   └── controller_exceptions.hpp
├── checker/
│   ├── simple_goal_checker.*     # XY + 航向
│   ├── position_goal_checker.*   # 仅 XY
│   ├── stopped_goal_checker.*    # XY + 航向 + 停止
│   ├── simple_progress_checker.* # 位移进度
│   └── pose_progress_checker.*   # 位移 + 转角进度
├── utils/
│   ├── velocity_smoother.*       # 速度平滑
│   ├── odometry_utils.*          # 里程计滑动平均
│   ├── controller_utils.*        # Lookahead、圆-线段交点
│   └── conversions.*             # Twist 2D/3D 转换
└── proto/
    ├── controller_options.proto
    ├── checker_options.proto
    └── smoother_options.proto
```

## 0.5 相关模块

| 模块 | 关系 |
|------|------|
| `autonomy/planning` | 提供全局路径；共享 costmap |
| `autonomy/map/costmap_2d` | 局部/全局代价地图、碰撞检测 |
| `autonomy/navigator` | 行为树调用 FollowPath |
| `autonomy/transform` | TF 位姿变换 |
| `autonomy/system` | `Autonomy` 统一构造 ControllerServer |

## 0.6 与 Nav2 对照

| Nav2 组件 | Autonomy 对应 | 备注 |
|-----------|---------------|------|
| `nav2_controller::ControllerServer` | `ControllerServer` | 骨架阶段 |
| `nav2_core::Controller` | `ControllerInterface` | API 对齐 |
| `nav2_core::GoalChecker` | `GoalChecker` | 三种实现已移植 |
| `nav2_core::ProgressChecker` | `ProgressChecker` | 两种实现已移植 |
| `nav2_velocity_smoother` | `VelocitySmoother` | 算法已移植 |
| `nav2_regulated_pure_pursuit_controller` | 工具函数就绪 | 插件待实现 |
| `nav2_dwb_controller` | 未实现 | — |
| `nav2_teb_local_planner` | 未实现 | — |
| `nav2_mppi_controller` | 配置预留 | 源码待引入 |
| `nav2_graceful_controller` | 配置预留 | 源码待引入 |

(control-overview)=
## 0.7 问题形式化

机器人状态 $x(t)=(x,y,\theta,v_x,v_y,\omega_z)^\top$，控制 $u=(v_x^{cmd},v_y^{cmd},\omega_z^{cmd})$，全局路径 $\mathcal{P}=\{p_i\}\subset SE(2)$ 由 `planning` 提供。局部控制可写为有限时域最优：

$$
\min_{u(\cdot)} J = \int_0^T \big( e_p^\top Q e_p + w_\theta e_\theta^2 + u^\top R u \big)\, dt
$$

约束：$(x,y)\in\mathcal{C}_{\mathrm{free}}$，速度/加速度界，终端 $p(T)\approx p_N$。航向误差 $e_\theta=\mathrm{AngleDiff}(\theta,\theta_{ref})$（结果 $\in(-\pi,\pi]$），Goal Checker 与控制器共用。

## 0.8 运动学概要

**差速（DiffDrive）**——Autonomy 默认：

$$
\dot{x}=v_x\cos\theta,\quad \dot{y}=v_x\sin\theta,\quad \dot{\theta}=\omega_z.
$$

离散（步长 $\Delta t$，MPPI `model_dt`）：$x_{k+1}=x_k+v_x\cos\theta_k\Delta t$，$y_{k+1}=y_k+v_x\sin\theta_k\Delta t$，$\theta_{k+1}=\theta_k+\omega_z\Delta t$。

| 模型 | 自由度 | Autonomy |
|------|--------|----------|
| DiffDrive | $v_x,\omega_z$ | 默认；MPPI `motion_model="DiffDrive"` |
| Holonomic | $v_x,v_y,\omega_z$ | MPPI 设 `vy_std>0` |
| Ackermann | $|\omega_z|\leq|v_x|/R_{\min}$ | `AckermannConstraints.min_turning_r` |

细节见 [§10 Graceful](controller/10_graceful_controller.md#3-运动模型)、[§11 MPPI](controller/11_mppi_controller.md#3-运动模型)、[§12 RPP](controller/12_rpp_controller.md#3-运动模型)、[§13 DWB](controller/13_dwb_controller.md#3-运动模型)。

## 0.9 控制流水线

$$
\boxed{\mathrm{Path}}
\xrightarrow{\mathrm{SetPlan}}
\boxed{\mathrm{Controller}}
\xrightarrow{f_{ctrl}}
\boxed{\mathrm{ComputeVelocityCommands}}
\xrightarrow{\mathrm{GoalChecker}}
\boxed{\mathrm{VelocitySmoother}}
\to \mathrm{cmd\_vel}
$$

并行：$\boxed{\mathrm{ProgressChecker}}\xrightarrow{\text{无进度}} \mathrm{FailedToMakeProgress}$。FollowPath 时序见 [§2.3](02_architecture.md#23-followpath-控制循环)。

## 0.10 算法插件一览

| 算法 | 核心公式 / 机制 | 专题 | 状态 |
|------|-----------------|------|------|
| Graceful | Lyapunov 平滑律 + motion target | [§10](controller/10_graceful_controller.md) | ⏳ 配置 |
| MPPI | $\mathbf{u}^*=\sum_k w_k \mathbf{u}^{(k)}$ | [§11](controller/11_mppi_controller.md) | ⏳ 配置 |
| RPP | $\kappa=2y_l/L_d^2$ + 线速度调节 | [§12](controller/12_rpp_controller.md) | 工具 ✅ |
| DWB | $\arg\min C_{total}(v,\omega)\in\mathcal{V}_{legal}$ | [§13](controller/13_dwb_controller.md) | ❌ |
| TEB | $\arg\min \tilde{V}(B)$（稀疏 WNLS） | [§14](controller/14_teb_controller.md) | ❌ |
| MPC | $\arg\min J(\mathbf{U})$ s.t. OCP | [§15](controller/15_mpc_controller.md) | ❌ |
| VelocitySmoother | $v_{out}=v_{curr}+\mathrm{clamp}(\eta\Delta v)$ | [§4](04_velocity_smoother.md) | 算法 ✅ |

综述与选型见 [§6](06_survey.md)。

## 0.11 快速开始

### 0.11.1 三步启用

1. 编辑 `config/control/controller.lua`
2. 在 `config/autonomy.lua` 中设置 `control = AUTONOMY_CONTROLLER`
3. 启动后 `system::Autonomy` 自动构造 `ControllerServer`

### 0.11.2 最小配置

```lua
-- config/control/controller.lua
AUTONOMY_CONTROLLER = {
    controller_frequency = 20.0,
    failure_tolerance = 30.0,
    publish_zero_velocity = false,

    controller_plugins = {
        -- "id:ClassName" 格式，待插件实现后启用
        -- "graceful_controller:GracefulController",
    },

    goal_checker = {
        xy_goal_tolerance = 0.25,
        yaw_goal_tolerance = 0.35,
        stateful = true,
    },
    progress_checker = {
        required_movement_radius = 0.5,
        movement_time_allowance = 10.0,
    },

    -- 附加模式：共享 planner 全局 costmap
    costmap = { enabled = false },
}
```

```lua
-- config/autonomy.lua
AUTONOMY = {
    control = AUTONOMY_CONTROLLER,
}
```

### 0.11.3 C++ 直接使用

```cpp
#include "autonomy/control/controller_server.hpp"
#include "autonomy/control/control_options.hpp"

auto dict = autonomy::common::LuaParameterDictionary::NonReferenceCounted(
    "config/control/controller.lua", autonomy::common::LoadLuaScript);
auto options = autonomy::control::LoadOptions(dict->GetDictionary("AUTONOMY_CONTROLLER").get());

auto server = std::make_shared<autonomy::control::ControllerServer>(options);
server->Start();
server->SetSharedCostmap(planner_server->GetCostmapWrapper());
server->UpdateOdometry(odom_msg);
// FollowPath（待 ComputeControl 完整实现）
// server->ComputeControl();
```

### 0.11.4 独立使用 Checker

```cpp
#include "autonomy/control/checker/simple_goal_checker.hpp"

auto checker = std::make_shared<autonomy::control::checker::SimpleGoalChecker>();
checker->Initialize("goal_checker", nullptr);
checker->SetTolerances(0.25, 0.35, true);

commsgs::geometry_msgs::Pose query, goal;
commsgs::geometry_msgs::Twist vel;
bool reached = checker->IsGoalReached(query, goal, vel);
```

### 0.11.5 独立使用 VelocitySmoother

```cpp
#include "autonomy/control/utils/velocity_smoother.hpp"

autonomy::control::proto::VelocitySmootherOptions opts;
opts.set_smoothing_frequency(20.0);
opts.set_scale_velocities(true);
opts.add_max_velocity(0.5);
opts.add_max_velocity(0.0);
opts.add_max_velocity(2.5);

autonomy::control::utils::VelocitySmoother smoother(opts);
// 输入命令后调用 smootherTimer() 获取平滑输出
```

### 0.11.6 当前限制

| 功能 | 状态 |
|------|------|
| FollowPath 完整循环 | 未实现 |
| 控制器插件加载 | 未实现 |
| `cmd_vel` 发布 | 未接线 |
| Checker 参数从 Lua 加载 | 未接线（硬编码默认值） |
| VelocitySmoother 定时器节点 | 未接线 |

## 0.12 配置

(control-usage)=

**入口文件**：`config/control/controller.lua` → `config/autonomy.lua`

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

**Goal Checker 配置**：

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

> **注意**：Lua 中 costmap 键名为 `costmap`，而 `LoadOptions()` 读取 `costmap_2d_options`。附加模式下通过 `SetSharedCostmap()` 共享 planner 全局地图。

## 0.13 ControllerServer API

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

## 0.14 ControllerInterface 插件 API

| 方法 | 说明 |
|------|------|
| `ComputeVelocityCommands(pose, vel, cmd_vel, goal_checker, msg)` | 核心：计算速度命令 |
| `SetPlan(path)` | 接收/更新全局路径 |
| `IsGoalReached(dist_tol, angle_tol)` | 控制器内部终点判定 |
| `SetSpeedLimit(speed, percentage)` | 动态限速 |
| `Reset()` | 导航结束重置状态 |

**返回值** `ControllerResultCode`：`0` SUCCESS · `102` NO_VALID_CMD · `104` COLLISION · `106` ROBOT_STUCK · `111` INVALID_PATH · `112` TF_ERROR。

## 0.15 系统集成

```text
controller_ = std::make_shared<control::ControllerServer>(options_.controller_options());
controller_->SetSharedCostmap(planner_->GetCostmapWrapper());
controller_->Start();
sensor_consumer->OnOdometry() → controller_->UpdateOdometry(odom);
```

```
用户目标 → Navigator BT → FollowPath → ControllerServer
                              ↑              ↓
                        Planning.Path    cmd_vel → 底盘
                              ↑
                        global costmap（共享）
```

## 0.16 通信接口（设计意图）

| 类型 | 名称 | 说明 |
|------|------|------|
| 节点 | `controller_server` | autolink 节点 |
| 话题 | `cmd_vel` | 速度输出（待接线） |
| 话题 | `odom` | 里程计输入 |
| Action | `FollowPath` | 行为树 FollowPath 节点 |
| 代价地图 | `/local_costmap` 或共享 `/global_costmap` | 碰撞检测 |

## 0.17 自定义控制器插件

1. 继承 `common::ControllerInterface`
2. 实现 `ComputeVelocityCommands()`、`SetPlan()` 等虚函数
3. `AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(MyController, ControllerInterface)`
4. 在 `controller.lua` 的 `controller_plugins` 中注册

参考 [§2.4 插件加载](02_architecture.md#24-插件与配置)。

## 0.18 控制器选型

| 场景 | 推荐 | 关键配置 |
|------|------|----------|
| 室内差速、窄通道 | Graceful Controller | `use_collision_detection=true` |
| 动态避障、复杂环境 | MPPI | `batch_size=2000`, CostCritic |
| 简单跟踪、低算力 | RPP | 小 lookahead |
| 高精度停车 | StoppedGoalChecker | `trans/rot_stopped_velocity` |
| 只关心位置 | PositionGoalChecker | 忽略航向 |

更多场景见 [06_survey §6.14](06_survey.md#614-工程选型矩阵)。

## 0.19 故障排查

| 异常 / 现象 | 常见原因 | 处理 |
|-------------|----------|------|
| `FailedToMakeProgress` | 机器人卡住、路径被挡 | 增大 `required_movement_radius` 或检查 costmap |
| `NoValidControl` | 控制器无法生成有效命令 | 检查路径有效性、costmap 更新 |
| `ControllerTFError` | TF 不可用 | 检查 `global_frame` / `base_link` |
| `InvalidPath` | 空路径或路径点过少 | 确认 planning 输出 |
| 控制命令始终为零 | `ComputeControl` 未实现 | 当前已知限制，待开发 |
| Checker 参数不生效 | Lua 未接线到 C++ | 使用 `SetTolerances()` 手动设置 |
| 局部 costmap 未加载 | Lua 键名 `costmap` vs `costmap_2d_options` | 使用 `SetSharedCostmap()` |

**检查清单**：`GetRobotPose()` 返回 true · odom 正常更新 · costmap `isCurrent()` · `controller_frequency` 与 `model_dt` 一致。

## 0.20 性能建议

| 建议 | 说明 |
|------|------|
| 控制频率 20 Hz | 与 MPPI `model_dt=0.05` 对齐 |
| 共享全局 costmap | 附加模式减少重复计算 |
| 启用 VelocitySmoother | 保护硬件、平滑加速度 |
| MPPI batch_size | 2000 适合桌面 CPU；嵌入式可降至 500–1000 |
| failure_tolerance | 短时 TF 抖动设 5–30 s |
