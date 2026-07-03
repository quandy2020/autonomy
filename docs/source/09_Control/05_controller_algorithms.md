# 5. 局部控制器算法

本文是局部控制器**总览与索引**；各算法六段式推导见 `controller/` 专题（§10–§15，文件名前缀 `10_*`–`15_*`）。当前仓库**尚无具体插件实现**，但 `controller.lua` 已含配置模板，几何工具（lookahead、圆-线段交点）已就绪。

> 选型与历史见 [06_survey.md](06_survey.md)；Checker 见 [03_checkers.md](03_checkers.md)；VelocitySmoother 见 [04_velocity_smoother.md](04_velocity_smoother.md)。

## 5.1 算法总览

| 算法 | 类型 | 专题 | 实现状态 |
|------|------|------|----------|
| [Graceful](controller/10_graceful_controller.md) | 几何 + Lyapunov 平滑 | [§10](controller/10_graceful_controller.md) | ⏳ 配置预留 |
| [MPPI](controller/11_mppi_controller.md) | 采样型 MPC | [§11](controller/11_mppi_controller.md) | ⏳ 配置预留 |
| [RPP](controller/12_rpp_controller.md) | 几何 + 调节 | [§12](controller/12_rpp_controller.md) | ⏳ 工具就绪 |
| [DWB](controller/13_dwb_controller.md) | 速度空间采样 | [§13](controller/13_dwb_controller.md) | ❌ 未实现 |
| [TEB](controller/14_teb_controller.md) | 时空优化 | [§14](controller/14_teb_controller.md) | ❌ 未实现 |
| [MPC](controller/15_mpc_controller.md) | 约束 OCP / NMPC | [§15](controller/15_mpc_controller.md) | ❌ 未实现 |

## 5.2 Graceful Controller

Nav2 `nav2_graceful_controller` 的平滑路径跟踪器。完整推导见 **[§10 Graceful Controller](controller/10_graceful_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Park & Kuipers, ICRA 2011 平滑控制律 |
| 特点 | motion target、减速区、可选碰撞仿真 |
| Autonomy | ⏳ 配置预留 |

## 5.3 MPPI Controller

Nav2 `nav2_mppi_controller` 随机 MPC 控制器。完整推导见 **[§11 MPPI Controller](controller/11_mppi_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Williams et al. 路径积分 MPC (2017/2018) |
| 核心 | batch 采样 + softmax + Critics |
| Autonomy | ⏳ `controller.lua` 已配置 |

## 5.4 Regulated Pure Pursuit（RPP）

Nav2 默认几何跟踪控制器。完整推导见 **[§12 RPP Controller](controller/12_rpp_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Coulter 1992 Pure Pursuit + Nav2 Regulated 扩展 |
| 工具 | `GetLookAheadPoint` 等已就绪 |
| Autonomy | ⏳ 插件未实现（Phase 1 推荐） |

## 5.5 DWB（Dynamic Window Benchmark）

Nav2 `nav2_dwb_controller` 在 Fox (1997) DWA 之上以 Lu (2014) Critic 框架工程化。完整推导见 **[§13 DWB Controller](controller/13_dwb_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Fox 1997 DWA → Lu 2014 TrajectoryCritic |
| 核心 | $\mathcal{V}_{legal}$ 网格采样 + $\arg\min C_{total}$ |
| Autonomy | ❌ 未实现 |

## 5.6 TEB（Timed Elastic Band）

时空弹性带优化局部规划器。完整推导见 **[§14 TEB Controller](controller/14_teb_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Rösmann et al. 2017 TEB |
| 核心 | 位姿序列 + $\Delta T_k$ 稀疏 WNLS |
| Autonomy | ❌ 未实现 |

## 5.7 模型预测控制（MPC）

经典 **NMPC**（约束有限时域 OCP + SQP/RTI 求解），与 §5.3 **MPPI**（采样型 MPC）区分。完整推导见 **[§15 MPC Controller](controller/15_mpc_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Rawlings & Mayne (2009)；移动机器人 NMPC |
| 与 MPPI | 显式 NLP/QP vs 路径积分采样 |
| Autonomy | ❌ 未实现 |

## 5.8 控制器对比

| 维度 | Graceful | MPPI | RPP | DWB | TEB | MPC |
|------|----------|------|-----|-----|-----|-----|
| 计算量 | 低 | 高 | 低 | 中 | 中–高 | 中–高 |
| 动态避障 | 中 | 强 | 弱 | 强 | 强 | 强* |
| 参数数量 | 少 | 多 | 中 | 多 | 多 | 多 |
| 路径跟踪 | 高 | 高 | 中 | 中 | 中 | 高 |
| 倒车 | ❌ | ✅ | ❌ | 可选 | ✅ | 可选 |
| Autonomy | ⏳ | ⏳ | ⏳ | ❌ | ❌ | ❌ |

\* 依赖约束/软约束建模质量。

## 5.9 实现路线建议

1. **Phase 1**：Regulated Pure Pursuit（`controller_utils` 已就绪，最低成本验证 FollowPath）
2. **Phase 2**：Graceful Controller（配置已就绪）
3. **Phase 3**：MPPI（计算密集，建议可选编译）
4. **Phase 4**：按场景评估 DWB / TEB / NMPC

## 5.10 自定义控制器开发清单

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
