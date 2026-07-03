# 5. 局部控制器算法

本文是局部控制器**总览与索引**；各算法六段式推导见 `controller/` 专题（§10–§15，文件名前缀 `10_*`–`15_*`）。当前仓库**尚无具体插件实现**，但 `controller.lua` 已含配置模板，几何工具（lookahead、圆-线段交点）已就绪。

> **选型**：工程速查 [§0.9.1](00_guide.md#091-局部轨迹与时空联合选型) · 谱系与决策树 [§6](06_survey.md)（[§6.6.4 时空联合](06_survey.md#664-时空联合轨迹规划)） · Checker [§3](03_checkers.md) · Smoother [§4](04_velocity_smoother.md)

## 5.1 算法总览

| 算法 | 轨迹层 | 类型 | 专题 | 实现状态 |
|------|--------|------|------|----------|
| [RPP](controller/12_rpp_controller.md) | 几何跟踪 | 几何 + 调节 | [§12](controller/12_rpp_controller.md) | ⏳ 工具就绪 |
| [Graceful](controller/10_graceful_controller.md) | 几何 + 启发式 time-scaling | Lyapunov 平滑 | [§10](controller/10_graceful_controller.md) | ⏳ 配置预留 |
| [MPPI](controller/11_mppi_controller.md) | 滚动 rollout $\tau_{0:H}$ | 采样型 MPC | [§11](controller/11_mppi_controller.md) | ⏳ 配置预留 |
| [DWB](controller/13_dwb_controller.md) | 单步 / 短 horizon rollout | 速度空间采样 | [§13](controller/13_dwb_controller.md) | ❌ 未实现 |
| [TEB](controller/14_teb_controller.md) | **显式** $s_k,\,\Delta T_k$ | 时空联合 NLP | [§14](controller/14_teb_controller.md) | ❌ 未实现 |
| [MPC](controller/15_mpc_controller.md) | 固定 $H\Delta t$ 网格 | 约束 OCP / NMPC | [§15](controller/15_mpc_controller.md) | ❌ 未实现 |

轨迹层定义见 [§0.7 局部轨迹三层](00_guide.md#07-问题形式化) 与 [§6.2.4](06_survey.md#624-几何路径轨迹与局部轨迹规划)。

## 5.2 Graceful Controller

Nav2 `nav2_graceful_controller` 的平滑路径跟踪器。完整推导见 **[§10 Graceful Controller](controller/10_graceful_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Park & Kuipers, ICRA 2011 平滑控制律 |
| 轨迹层 | 几何跟踪 + `slowdown_radius` 等**启发式 time-scaling**（非 TEB 式 $\Delta T_k$） |
| 特点 | motion target、减速区、可选碰撞仿真 |
| Autonomy | ⏳ 配置预留 · Phase 2 |

## 5.3 MPPI Controller

Nav2 `nav2_mppi_controller` 随机 MPC 控制器。完整推导见 **[§11 MPPI Controller](controller/11_mppi_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Williams et al. 路径积分 MPC (2017/2018) |
| 轨迹层 | 固定 $\Delta t$ rollout；隐式离散时空轨迹 $\tau_{0:H}$ |
| 核心 | batch 采样 + softmax + Critics |
| Autonomy | ⏳ `controller.lua` 已配置 · Phase 3 |

## 5.4 Regulated Pure Pursuit（RPP）

Nav2 默认几何跟踪控制器。完整推导见 **[§12 RPP Controller](controller/12_rpp_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Coulter 1992 Pure Pursuit + Nav2 Regulated 扩展 |
| 轨迹层 | 纯几何；每周期 $(v,\omega)$，不显式存 $\tau(t)$ |
| 工具 | `GetLookAheadPoint` 等已就绪 |
| Autonomy | ⏳ 插件未实现 · **Phase 1 推荐** |

## 5.5 DWB（Dynamic Window Benchmark）

Nav2 `nav2_dwb_controller` 在 Fox (1997) DWA 之上以 Lu (2014) Critic 框架工程化。完整推导见 **[§13 DWB Controller](controller/13_dwb_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Fox 1997 DWA → Lu 2014 TrajectoryCritic |
| 轨迹层 | 在 $\mathcal{V}_{legal}$ 上采样 $(v,\omega)$，短 rollout 隐式 $\tau$；**无**显式 $\Delta T_k$ |
| 核心 | $\mathcal{V}_{legal}$ 网格采样 + $\arg\min C_{total}$ |
| 选型 | 动态避障、可组合 critic；Autonomy 优先 **MPPI**（Nav2 官方新栈） |
| Autonomy | ❌ 未实现 |

## 5.6 TEB（Timed Elastic Band）

Nav2 生态最典型的**显式时空联合**局部规划器（第三方 `teb_local_planner`）。完整推导见 **[§14 TEB Controller](controller/14_teb_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Rösmann et al. 2009–2017；EB → Timed EB 演进见 [§6.6.5](06_survey.md#665-elastic-band-与时空参数化演进) |
| 轨迹层 | 优化 $B=\{s_k,\,\Delta T_k\}$；$v_k,a_k$ 由 $\Delta T_k$ 导出 |
| 核心 | 稀疏 WNLS + LM；动态障碍可接 [Prediction](../11_Prediction/08_behavior_prediction.md) |
| Checker | 精密停靠配 [StoppedGoalChecker](checker/17_stopped_goal_checker.md)（[§0.9.1](00_guide.md#091-局部轨迹与时空联合选型)） |
| Autonomy | ❌ 未实现 |

## 5.7 模型预测控制（MPC）

经典 **NMPC**（约束有限时域 OCP + SQP/RTI 求解），与 §5.3 **MPPI**（采样型 MPC）区分。完整推导见 **[§15 MPC Controller](controller/15_mpc_controller.md)**。

| 要点 | 说明 |
|------|------|
| 理论 | Rawlings & Mayne (2009)；移动机器人 NMPC |
| 轨迹层 | 固定 horizon 网格 $\{x_k,u_k\}$；离散时空轨迹，非 TEB 式 $\Delta T_k$ 变量 |
| 与 MPPI | 显式 NLP/QP vs 路径积分采样 |
| 与 TEB | 硬约束保证性 vs band 稀疏 NLP；泊车/高速更常 NMPC |
| Autonomy | ❌ 未实现 |

## 5.8 控制器对比

| 维度 | Graceful | MPPI | RPP | DWB | TEB | MPC |
|------|----------|------|-----|-----|-----|-----|
| 显式时间 | ❌ | 隐式 $\Delta t$ 固定 | ❌ | 隐式 | ✅ $\Delta T_k$ | 固定网格 |
| 计算量 | 低 | 高 | 低 | 中 | 中–高 | 中–高 |
| 动态避障 | 中 | 强 | 弱 | 强 | 强 | 强* |
| 参数数量 | 少 | 多 | 中 | 多 | 多 | 多 |
| 路径跟踪 | 高 | 高 | 中 | 中 | 中 | 高 |
| 倒车 | ❌ | ✅ | ❌ | 可选 | ✅ | 可选 |
| Autonomy | ⏳ | ⏳ | ⏳ | ❌ | ❌ | ❌ |

\* 依赖约束/软约束建模质量。

### 5.8.1 控制器与 Checker 配对

与 [§0.9.1](00_guide.md#091-局部轨迹与时空联合选型) 一致；Checker 六段式见 `checker/15_*`–`19_*`。

| 场景 | 控制器 | Goal Checker | Progress Checker |
|------|--------|--------------|------------------|
| 室内默认 | RPP / Graceful | Simple | Simple |
| 动态避障 | MPPI / DWB | Simple | Simple |
| 先转后走 | Graceful | Simple | **Pose** |
| 充电对接 | RPP / Graceful + Smoother | **Stopped** | Simple |
| 只到点 | 任意 | **Position** | Simple |
| TEB / NMPC | TEB / MPC | Stopped（推荐） | Pose（若多原地转） |

## 5.9 实现路线建议

1. **Phase 1**：Regulated Pure Pursuit + SimpleGoalChecker（`controller_utils` 已就绪）
2. **Phase 2**：Graceful + PoseProgressChecker（若初始旋转策略）
3. **Phase 3**：MPPI + SimpleGoalChecker（可选编译、GPU/多核）
4. **Phase 4**：按场景评估 DWB / **TEB**（显式时空）/ **NMPC**；对接类任务配 StoppedGoalChecker

与 [§6.5.2 Autonomy 谱系](06_survey.md#652-autonomy-在技术谱系中的位置) 及 Navigator FollowPath 接线同步推进。

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
