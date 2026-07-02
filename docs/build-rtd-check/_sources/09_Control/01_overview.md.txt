(control-overview)=
# 1. 模块概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 控制层级 | 局部轨迹跟踪（Local Controller / Trajectory Tracker） |
| 输入 | 全局路径 `planning_msgs::Path`、当前位姿、当前速度、局部/全局代价地图 |
| 输出 | `geometry_msgs::TwistStamped`（`cmd_vel`） |
| 上游 | `planning`（路径）、`localization`（位姿）、`map`（costmap） |
| 下游 | 底盘驱动、仿真器 |
| 对标 | nav2_controller、nav2_velocity_smoother |

### 1.2 核心能力

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

> **当前阶段**：Checker、Smoother、几何工具已就绪；`ControllerServer` 主循环与控制器插件（MPPI、Graceful 等）尚待实现。详见 [§5 架构](05_architecture.md#52-实现状态)。

### 1.3 源码结构

```
autonomy/control/
├── controller_server.*           # FollowPath 服务入口
├── control_options.*             # Lua → ControllerLabControllerOptions
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

### 1.4 导航栈数据流

```
Planning.Path ──→ ControllerServer ──→ cmd_vel ──→ 底盘
       ↑                ↑
  global costmap    local costmap（可选）
       ↑                ↑
   map 模块         传感器 / 障碍层
```

### 1.5 相关模块

| 模块 | 关系 |
|------|------|
| `autonomy/planning` | 提供全局路径；共享 costmap |
| `autonomy/map/costmap_2d` | 局部/全局代价地图、碰撞检测 |
| `autonomy/navigator` | 行为树调用 FollowPath |
| `autonomy/transform` | TF 位姿变换 |
| `autonomy/system` | `Autonomy` 统一构造 ControllerServer |

### 1.6 与 Nav2 对照

| Nav2 组件 | Autonomy 对应 | 备注 |
|-----------|---------------|------|
| `nav2_controller::ControllerServer` | `ControllerServer` | 骨架阶段 |
| `nav2_core::Controller` | `ControllerInterface` | API 对齐 |
| `nav2_core::GoalChecker` | `GoalChecker` | 三种实现已移植 |
| `nav2_core::ProgressChecker` | `ProgressChecker` | 两种实现已移植 |
| `nav2_velocity_smoother` | `VelocitySmoother` | 算法已移植 |
| `nav2_mppi_controller` | 配置预留 | 源码待引入 |
| `nav2_graceful_controller` | 配置预留 | 源码待引入 |
