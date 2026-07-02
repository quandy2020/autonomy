# Control 运动控制指南

`autonomy/control` 是 Autonomy 的**局部运动控制**子系统，对齐 ROS 2 Navigation2 `nav2_controller`。接收全局路径，以固定频率计算并发布 `cmd_vel`，直至目标到达。本文档为模块总入口。

**编号约定**：**§0–§9** 为模块主干（与 Planning、Map 等模块一致）；**§10–§20** 为组件/算法专题深读，分别对应 §6、§7、§8 中的具体实现。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§5 架构](05_architecture.md) → [§4 使用指南](04_usage.md) |
| 算法研发 | [§9 综述](09_survey.md) → [§3 数学原理](03_math.md) → [§8 局部控制器](08_controller_algorithms.md) → [§10–§14 控制器专题](#1014-局部控制器专题8-详读) |
| 集成调试 | [§2 快速开始](02_quickstart.md) → [§4 使用指南](04_usage.md) → [§6 检查器](06_checkers.md) → [§4.8 排错](04_usage.md#48-故障排查) |

### §0–§9 主干

| 编号 | 文件 | 内容 |
|------|------|------|
| 0 | [00_guide.md](00_guide.md) | 本指南（模块入口） |
| 1 | [01_overview.md](01_overview.md) | 模块概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_math.md](03_math.md) | 数学原理 |
| 4 | [04_usage.md](04_usage.md) | 使用指南 |
| 5 | [05_architecture.md](05_architecture.md) | 模块架构设计 |
| 6 | [checker/index.rst](checker/index.rst) → [06_checkers.md](06_checkers.md) | 目标与进度检查器（总览 + 专题） |
| 7 | [smoother/index.rst](smoother/index.rst) → [07_velocity_smoother.md](07_velocity_smoother.md) | 速度平滑器（总览 + 专题） |
| 8 | [controller/index.rst](controller/index.rst) → [08_controller_algorithms.md](08_controller_algorithms.md) | 局部控制器（总览 + 专题） |
| 9 | [09_survey.md](09_survey.md) | 运动控制算法综述 |

### §10–§14 局部控制器专题（§8 详读）

| 编号 | 文件 | 对应 §8 | 算法 |
|------|------|---------|------|
| 10 | [controller/10_graceful_controller.md](controller/10_graceful_controller.md) | §8.2 | Graceful Controller |
| 11 | [controller/11_mppi_controller.md](controller/11_mppi_controller.md) | §8.3 | MPPI Controller |
| 12 | [controller/12_rpp_controller.md](controller/12_rpp_controller.md) | §8.4 | Regulated Pure Pursuit |
| 13 | [controller/13_dwb_controller.md](controller/13_dwb_controller.md) | §8.5 | DWB Controller |
| 14 | [controller/14_teb_controller.md](controller/14_teb_controller.md) | §8.6 | TEB Controller |

### §15–§19 Checker 专题（§6 详读）

| 编号 | 文件 | 对应 §6 | 组件 |
|------|------|---------|------|
| 15 | [checker/15_simple_goal_checker.md](checker/15_simple_goal_checker.md) | §6.2 | SimpleGoalChecker |
| 16 | [checker/16_position_goal_checker.md](checker/16_position_goal_checker.md) | §6.3 | PositionGoalChecker |
| 17 | [checker/17_stopped_goal_checker.md](checker/17_stopped_goal_checker.md) | §6.4 | StoppedGoalChecker |
| 18 | [checker/18_simple_progress_checker.md](checker/18_simple_progress_checker.md) | §6.5 | SimpleProgressChecker |
| 19 | [checker/19_pose_progress_checker.md](checker/19_pose_progress_checker.md) | §6.6 | PoseProgressChecker |

### §20 Smoother 专题（§7 详读）

| 编号 | 文件 | 内容 |
|------|------|------|
| 20 | [smoother/20_velocity_smoother_impl.md](smoother/20_velocity_smoother_impl.md) | VelocitySmoother 源码级实现 |
