# 3　入门

结构对齐 [OCS2 Getting Started](https://leggedrobotics.github.io/ocs2/getting-started.html)；接口与路径以本仓库为准。

- 前提：已按 [02_installation.md](02_installation.md) 编出 `libbolza`
- 细则：代价 / 约束见 [05_optimal_control_modules.md](05_optimal_control_modules.md)；示例见 [04_robotic_examples.md](04_robotic_examples.md)

## 3.1　从哪里开始

先选定模型复杂度，再从相近示例改起（勿从零搭）：

| 模型 | 适用 | 示例 |
|:---|:---|:---|
| 运动学 | 固定 / 移动机械臂；末端跟踪、自碰撞、避障导航 | `mobile_manipulator_demo` |
| 全动力学 | 完整刚体动力学 | `ballbot_demo`、`cartpole_demo`、`quadrotor_demo` |
| 质心动力学 | 足式：全运动学 + 外力相关动力学（比全动力学便宜） | `legged_robot_demo` |

- 示例总表：[04_robotic_examples.md](04_robotic_examples.md)
- URDF 辅助：[08_from_urdf_to_ocp.md](08_from_urdf_to_ocp.md)

## 3.2　最优控制问题

Bolza 面向**切换系统**：有限个连续子系统，由时间触发事件切换。应用包括档位、换流、制造过程、步行接触切换等。

### 3.2.1　切换系统形式

$$
\begin{aligned}
\min_{\mathbf{u}(\cdot)}\quad
& \sum_i \phi_i\big(\mathbf{x}(t_{i+1})\big)
+ \int_{t_i}^{t_{i+1}} l_i\big(\mathbf{x}(t),\mathbf{u}(t),t\big)\,dt \\
\text{s.t.}\quad
& \mathbf{x}(t_0)=\mathbf{x}_0 \\
& \dot{\mathbf{x}}(t)=\mathbf{f}_i\big(\mathbf{x}(t),\mathbf{u}(t),t\big) \\
& \mathbf{x}(t_{i+1}^+)=\mathbf{j}\big(\mathbf{x}(t_{i+1})\big) \\
& \mathbf{g}_{1,i}\big(\mathbf{x}(t),\mathbf{u}(t),t\big)=\mathbf{0} \\
& \mathbf{g}_{2,i}\big(\mathbf{x}(t),t\big)=\mathbf{0} \\
& \mathbf{h}_i\big(\mathbf{x}(t),\mathbf{u}(t),t\big)\ge\mathbf{0} \\
& t_i < t < t_{i+1},\quad i\in\{0,1,\ldots,I-1\}
\end{aligned}
$$

- $t_i$：切换时刻；$t_I$：终端时刻
- 每模式可有各自代价 / 动力学 / 约束；段间靠跳映射衔接
- 各段状态 / 输入维数可不同，须正确给出跳映射
- 须事先给定**模式序列**与**事件时间**（时间触发）

### 3.2.2　单模式简化

四旋翼、ballbot、cartpole 等仅一个模式时，退化为常规定问题：

$$
\begin{aligned}
\min_{\mathbf{u}(\cdot)}\quad
& \phi\big(\mathbf{x}(t_I)\big)
+ \int_{t_0}^{t_I} l\big(\mathbf{x}(t),\mathbf{u}(t),t\big)\,dt \\
\text{s.t.}\quad
& \mathbf{x}(t_0)=\mathbf{x}_0 \\
& \dot{\mathbf{x}}(t)=\mathbf{f}\big(\mathbf{x}(t),\mathbf{u}(t),t\big) \\
& \mathbf{g}_1\big(\mathbf{x}(t),\mathbf{u}(t),t\big)=\mathbf{0} \\
& \mathbf{g}_2\big(\mathbf{x}(t),t\big)=\mathbf{0} \\
& \mathbf{h}\big(\mathbf{x}(t),\mathbf{u}(t),t\big)\ge\mathbf{0}
\end{aligned}
$$

## 3.3　如何定义 OCP

至少需要：

- 动力学 `SystemDynamicsBase`（或 `*AD`）
- 代价：对 $u$ 的 Hessian **正定**；对 $x$ **半正定**

容器为 `bolza::OptimalControlProblem`（`bolza/oc/oc_problem/optimal_control_problem.hpp`），主要成员：

**代价**

- `costPtr` — 中间 $(t,x,u)$
- `stateCostPtr` — 中间 $(t,x)$
- `preJumpCostPtr` — 切换前
- `finalCostPtr` — 终端

**软约束（罚函数）**

- `softConstraintPtr` / `stateSoftConstraintPtr`
- `preJumpSoftConstraintPtr` / `finalSoftConstraintPtr`

**硬约束**

- `equalityConstraintPtr` — 状态-输入等式（对 $u$ 的 Jacobian 须行满秩）
- `stateEqualityConstraintPtr` — 纯状态等式
- `inequalityConstraintPtr` / `stateInequalityConstraintPtr`
- `preJump*` / `final*` — 切换前与终端约束

**其它**

- `dynamicsPtr` — 动力学
- `preComputationPtr` — 预计算缓存
- 增广拉格朗日集合：`*LagrangianPtr`（按需）

- 装配细则：[05_optimal_control_modules.md](05_optimal_control_modules.md)

## 3.4　如何搭 MPC 回路

开环 OCP 定义后，须在控制频率上**反复**用最新观测求解。算力不足时，需要：

1. MPC 侧尽快用最新状态重解
2. 控制侧安全读取最新策略（避免读写竞争）

Bolza 提供 **MPC** 与 **MRT**（Model Reference Tracking）配合完成上述同步。同进程常用 `MPCMRTInterface`（`bolza/mpc/mpc_mrt_interface.hpp`）。

### 3.4.1　MPC

- 职责：用最新 `SystemObservation` 推进求解器
- 接口：`SetCurrentObservation` → `AdvanceMpc`
- 若上一次求解未结束，观测写入深度为 1 的缓冲，始终取最新状态

### 3.4.2　MRT

- 职责：安全读取求解结果
- **时间视图**：按查询时刻对优化轨迹线性插值，得 $(x,u)$
- **状态视图**：用反馈策略 $u=u_{ff}+K(x-\bar x)$（须在 settings 中打开 `useFeedbackPolicy`，且求解器支持）
- 常用：`updatePolicy` 后，再按时间或状态取控制

### 3.4.3　进程与通信（相对 OCS2）

| OCS2 | Bolza |
|:---|:---|
| 同机：`MpcMrtInterface` | 同机：`MPCMRTInterface`（示例默认） |
| 分机：`MpcRosInterface` + `MrtRosInterface` | 库无分机 / auto* 通信层 |

- 示例循环：`examples/runtime/demo_loop.hpp` / `.cpp`

## 3.5　链接与最小代码

```cmake
target_link_libraries(my_node PUBLIC bolza)
```

```cpp
#include <bolza/core/types.hpp>
#include <bolza/oc/oc_problem/optimal_control_problem.hpp>
#include <bolza/ddp/gauss_newton_ddp_mpc.hpp>
#include <bolza/mpc/mpc_mrt_interface.hpp>
#include <bolza/tools/common/robot_interface.hpp>
#include <bolza/conf/conf.hpp>
```

**纯库解题**（推荐）：

```cpp
#include <bolza/solver.hpp>
bolza::Problem problem;
problem.SetLinearDynamics(A, B);
problem.AddQuadraticCost(Q, R);
problem.AddQuadraticFinalCost(Qf);
problem.SetTarget(x_star);
problem.SetInitialState(x0);
bolza::Solver::Options options;  // options.tf 开环；options.mpc_steps>0 滚动
bolza::Solver::Summary summary;
bolza::Solve(options, &problem, &summary);
```

教程：[04_robotic_examples.md](04_robotic_examples.md) §4.1 · `examples/tutorial/solve_double_integrator.cpp`（`bolza_tutorial_di`）。  
底层仍是 `OptimalControlProblem` + `SLQ` / `GaussNewtonDDPMPC`（facade 内部调用）。

对照 `examples/cartpole`（预装 OCP 的 `CartPole`）：

1. 继承 `RobotInterface`：读 `task.info`，经 `bolza::Problem` 建动力学 / OCP / Rollout / Initializer
2. 构造 `GaussNewtonDDPMPC`（或 `SqpMpc` / `IpmMpc` / `SlpMpc`）
3. 包一层 `MPCMRTInterface`；`SetTargetTrajectories`（切换系统另设 mode schedule）
4. 循环：`SetCurrentObservation` → `AdvanceMpc` → `UpdatePolicy` → `RolloutPolicy`

```cpp
bolza::GaussNewtonDDPMPC mpc(
    interface.MpcSettings(), interface.DdpSettings(),
    interface.GetRollout(), interface.GetOptimalControlProblem(),
    interface.GetInitializer());
bolza::MPCMRTInterface mrt(mpc);
mrt.InitRollout(&interface.GetRollout());
mrt.GetReferenceManager().SetTargetTrajectories(target);
mrt.SetCurrentObservation(obs);
mrt.AdvanceMpc();
mrt.UpdatePolicy();
```

常用键（见 [06_configuration.md](06_configuration.md)）：

- `mpc.timeHorizon` — 预测时域（秒）
- `mpc.mrtDesiredFrequency` / `mpc.mpcDesiredFrequency` — MRT / 求解频率
- `ddp.maxNumIterations` — 实时常取 $1$ + 热启动
- `ddp.useFeedbackPolicy` — 是否启用状态视图反馈

## 3.6　如何检验 MPC 输出

调代价与超参时，先把「规划」与「跟踪执行」分开：

1. **空跑可视化**（可选）：MRT 插值轨迹；上层 demo 自行对接显示
2. **同模型 rollout**：给 MRT 设置与 OCP 相同的 `Rollout`（**仅需 `libbolza`**）
3. **更高保真仿真**：外部仿真器作 rollout（上游常用 RaiSim）

- 示例进程即上述 1–2 的集成：[04_robotic_examples.md](04_robotic_examples.md)
- 求解器选型：[07_solvers.md](07_solvers.md)
- 排障：[13_faq.md](13_faq.md)
