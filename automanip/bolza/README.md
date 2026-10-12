# Bolza

面向**连续时间最优控制与实时 MPC** 的 C++ 库（`libbolza`）。

独立 CMake 包：不依赖 ROS / autonomy / autolink。可对本目录单独配置，也可由 [`automanip`](../) `add_subdirectory`。

## 解决什么问题

机器人与动态系统上的在线控制，需要反复求解带约束的轨迹优化：

$$
\min_{x(\cdot),\,u(\cdot)}\; \Phi\!\bigl(x(t_f)\bigr) + \int_{t_0}^{t_f} L\!\bigl(x,u,t\bigr)\,dt
\quad\text{s.t.}\quad
\dot x = f(x,u),\quad
g(x,u)\le 0,\quad
\text{边界 / 切换模式}
$$

Bolza 把这类 **Bolza 形式 OCP** 建成可微模型，并用 DDP / 多重射击在毫秒～几十毫秒级滚动求解，供 MPC 闭环使用。

| 痛点 | Bolza 做法 |
|------|------------|
| 非线性动力学 + 二次（或更一般）代价 | `bolza::Problem` / `OptimalControlProblem` + CppAD |
| 路径 / 输入约束 | 增广拉格朗日、松弛障碍；SQP·IPM 子问题走 HPIPM |
| 足式等切换接触 | `switched_model` + 模式调度 |
| URDF 到 OCP | Pinocchio 运动学 / 动力学 / 自碰撞（可选） |
| 无中间件部署 | 纯库 API；可视化与通信留给上层（如 `automanip/examples`） |

算法源自 [OCS2](https://leggedrobotics.github.io/ocs2/)（BSD），命名空间与构建已改为 `bolza` / CMake。许可细节见文末。

## 怎么用（最小路径）

```cpp
bolza::Problem problem;
problem.SetLinearDynamics(A, B);       // 或 SetDynamics(...)
problem.AddQuadraticCost(Q, R);
problem.AddQuadraticFinalCost(Qf);
problem.SetInitialState(x0);
problem.SetTarget(x_star);

bolza::Solver::Options options;
bolza::Solver::Summary summary;
bolza::Solve(options, &problem, &summary);   // 开环或短时域 MPC
```

- 教程：`examples/tutorial/solve_double_integrator.cpp` → 目标 `bolza_tutorial_di`
- 带可视化的机器人 demo：[`automanip/examples`](../examples/)（`double_integrator` / `cartpole` / `quadrotor` / `ballbot` / …）
- 完整文档：[docs/README.md](docs/README.md)

## 求解器与模块（重点）

| 能力 | 位置 |
|------|------|
| DDP 族（Gauss-Newton / SLQ / iLQR） | `ddp/` |
| 多重射击 SQP · IPM · SLP | `sqp/` `ipm/` `slp/` |
| MPC + MRT 运行时 | `mpc/` |
| 建模原语（cost / constraint / AD / rollout） | `core/` `oc/` |
| Pinocchio · 切换模型 · MPC-Net | `pinocchio/` `switched_model/` `mpcnet/` |

头文件：`#include <bolza/...>` · 目标：`bolza` / `bolza::bolza` · 版本：`#include <bolza/conf/conf.hpp>`。

## 构建

**必选依赖**：Eigen3、OpenMP、CppAD/Cg、Pinocchio、GLPK、BLASFEO + HPIPM。  
**可选**：coal（自碰撞）、ONNX Runtime（`BOLZA_WITH_ONNX`）。

```bash
cmake -S automanip/bolza -B build/bolza \
  -DCMAKE_PREFIX_PATH=/path/to/cppad/prefix
cmake --build build/bolza -j"$(nproc)" --target bolza
```

| 选项 | 默认 | 说明 |
|------|------|------|
| `BOLZA_WITH_ONNX` | ON | 找到 onnxruntime 时编入 ONNX 控制器 |
| `BOLZA_BUILD_TESTS` | OFF | 注册 `bolza/**/*_test.cpp` |

Demo / 教程可执行文件在上层 `automanip` umbrella 中构建，不链入本库目标本身。

## 许可证

- 包元数据 / 本仓库新增文件：Apache-2.0
- `bolza/` 算法源码：上游 **BSD-3-Clause**（OCS2），见 [`solver_license.txt`](solver_license.txt)
- BLASFEO / HPIPM：上游 BSD-2-Clause
