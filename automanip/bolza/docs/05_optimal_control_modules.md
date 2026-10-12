# 5　最优控制模块

- 问题容器：`OptimalControlProblem`（`bolza/oc/oc_problem/optimal_control_problem.hpp`）
- 时间点：intermediate · prejump（切换前）· final
- 注册：按名称；可用 `isActive` 开关
- 运行中改参：仅经 `ReferenceManager` / `SolverSynchronizedModule`（勿改求解器内部拷贝）

## 5.1　代价

| 时间点 | 成员 | 自变量 |
|:---|:---|:---|
| intermediate | `costPtr` / `stateCostPtr` | $(t,x,u)$ / $(t,x)$ |
| prejump、final | `preJumpCostPtr`、`finalCostPtr` 等 | $(t,x)$ |

- 须提供：数值 + 二次近似
- 简单二次：`QuadraticStateCost` / `QuadraticStateInputCost`
- 复杂映射：`*CppAd`
- $\lVert f(x,u)\rVert^2$：`StateInputCostGaussNewtonAd`
- Hessian：对 $u$ 之和须正定；对 $x$ 宜半正定
- 目录：`bolza/core/cost/`

## 5.2　约束

- 基类：`StateConstraint` / `StateInputConstraint`（及 `*CppAd`）
- 软约束：罚函数 → `State*SoftConstraint` → soft 容器
- 状态–输入等式：投影；对 $u$ 的 Jacobian 须行满秩，否则改软约束
- 纯状态等式 / 不等式：松弛障碍或增广拉格朗日（`bolza/core/augmented_lagrangian/`）

## 5.3　动力学与预计算

- 流映射：$\dot{\mathbf{x}}=\mathbf{f}(\mathbf{x},\mathbf{u},t)$
- 接口：`SystemDynamicsBase` / `*AD`
- 求值前：`PreComputation::Request`（及 preJump / final）
- 可缓存：FK、碰撞等；宜先无缓存跑通，再对照 `mobile_manipulator`

## 5.4　参数同步

1. 唯一 `shared_ptr` 同步模块，注入代价 / 约束 / 动力学
2. `setReferenceManager` 或 `addSynchronizedModule`
3. `preSolverRun` 只做缓冲交换；重计算放其他线程

- `ReferenceManager`：`TargetTrajectories` · `ModeSchedule`
- 配置：[06_configuration.md](06_configuration.md)
- 求解器：[07_solvers.md](07_solvers.md)
