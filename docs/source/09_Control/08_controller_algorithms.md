# 8. 局部控制器算法

本文详述 Autonomy 配置中预留的局部控制器算法原理、数学公式与参数说明。当前仓库**尚无具体插件实现**，但 `controller.lua` 已包含完整配置模板，几何工具函数（lookahead、圆-线段交点）已就绪。

> 数学推导见 [03_math.md](03_math.md)；综述对比见 [09_survey.md](09_survey.md)。

## 8.1 算法总览

| 算法 | 类型 | 专题文档 | 实现状态 |
|------|------|----------|----------|
| [Graceful](controller/10_graceful_controller.md) | 几何 + 平滑 | §10 | ⏳ 配置预留 |
| [MPPI](controller/11_mppi_controller.md) | 随机 MPC | §11 | ⏳ 配置预留 |
| [RPP](controller/12_rpp_controller.md) | 几何 + 调节 | §12 | ⏳ 工具就绪 |
| [DWB](controller/13_dwb_controller.md) | 速度空间采样 | §13 | ❌ 未实现 |
| [TEB](controller/14_teb_controller.md) | 时空优化 | §14 | ❌ 未实现 |

## 8.2 Graceful Controller

Nav2 `nav2_graceful_controller` 的平滑路径跟踪器。完整架构、论文与逐步推导见 **[§10 Graceful Controller](controller/10_graceful_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Park & Kuipers, ICRA 2011 平滑控制律 |
| 特点 | 初始旋转、减速区、可选碰撞检测 |
| Autonomy | ⏳ 配置预留 |

## 8.3 MPPI Controller

Nav2 `nav2_mppi_controller` 随机 MPC 控制器。完整架构、论文、Critics 与配置见 **[§11 MPPI Controller](controller/11_mppi_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Williams et al. 路径积分 MPC (2017/2018) |
| 核心 | batch 采样 + softmax + Critics |
| Autonomy | ⏳ `controller.lua` 已配置 |

## 8.4 Regulated Pure Pursuit（RPP）

Nav2 默认几何跟踪控制器。Pure Pursuit 推导、Regulated 调节与 Autonomy 工具函数见 **[§12 RPP Controller](controller/12_rpp_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Coulter 1992 Pure Pursuit + Nav2 Regulated 扩展 |
| 工具 | `GetLookAheadPoint` 等已就绪 |
| Autonomy | ⏳ 插件未实现（Phase 1 推荐） |

## 8.5 DWB（Dynamic Window Benchmark）

完整文档见 **[§13 DWB Controller](controller/13_dwb_controller.md)**。

## 8.6 TEB（Timed Elastic Band）

时空弹性带优化局部规划器。完整文档见 **[§14 TEB Controller](controller/14_teb_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Rösmann et al. 2017 TEB |
| Autonomy | ❌ 未实现 |

## 8.7 控制器对比

| 维度 | Graceful | MPPI | RPP | DWB | TEB |
|------|----------|------|-----|-----|-----|
| 计算量 | 低 | 高 | 低 | 中 | 中–高 |
| 动态避障 | 中 | 强 | 弱 | 强 | 强 |
| 参数数量 | 少 | 多 | 中 | 多 | 多 |
| 路径跟踪 | 高 | 高 | 中 | 中 | 中 |
| 倒车 | ❌ | ✅ | ❌ | 可选 | ✅ |
| Autonomy | ⏳ | ⏳ | ⏳ | ❌ | ❌ |

## 8.8 实现路线建议

1. **Phase 1**：基于已有 `controller_utils` 实现 Regulated Pure Pursuit（最低成本、可快速验证 FollowPath 循环）
2. **Phase 2**：移植 Graceful Controller（配置已就绪）
3. **Phase 3**：引入 MPPI（计算密集，建议可选编译）
4. **Phase 4**：评估 DWB / TEB 需求

## 8.9 自定义控制器开发清单

```
□ 继承 ControllerInterface
□ 实现 ComputeVelocityCommands()
□ 实现 SetPlan() — 接收 planning_msgs::Path
□ 实现 IsGoalReached() — 委托 goal_checker 或内部判定
□ 实现 SetSpeedLimit()
□ 注册 AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN
□ 编写 controller.lua 配置段
□ 单元测试：直线路径、急转弯、障碍场景
```
