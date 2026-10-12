# 1　总览

- C++ 库，面向 Bolza 最优控制与切换系统 MPC
- 路径：`automanip/bolza/`（子工程或独立 `cmake -S`）
- 示例：`automanip/examples/`

## 1.1　库标识

- CMake 目标：`bolza` / `bolza::bolza`
- 命名空间：`bolza`
- 头文件：`#include <bolza/...>`
- 版本头：`#include <bolza/conf/conf.hpp>`（配置期生成）

## 1.2　求解器

- DDP 族：SLQ、iLQR
- 多重射击：SQP、IPM、SLP
- 路径约束：增广拉格朗日 / 松弛障碍
- SQP·IPM 二次子问题：HPIPM（`bolza/ipm/hpipm_interface.*`）
- 文档：[07_solvers.md](07_solvers.md)

## 1.3　建模与部署

- URDF → 运动学 / 动力学 / 自碰撞（Pinocchio、HPP-FCL，可选）
- 自动微分：CppAD / Cg
- 运行时：MPC、MRT
- 文档：[08_from_urdf_to_ocp.md](08_from_urdf_to_ocp.md)

## 1.4　源码分层

```text
mpc/ → ddp | sqp | ipm | slp | qp/ → oc/ → core/
       frank_wolfe · bilevel · switched_model
       tools · pinocchio · perceptive · mpcnet
```

- 文档：[11_architecture.md](11_architecture.md)

## 1.5　相对 OCS2

源自 [OCS2](https://leggedrobotics.github.io/ocs2/overview.html)。差异：

| | OCS2 | Bolza |
|:---|:---|:---|
| 构建 | catkin | CMake |
| 通信 / 显示 | ROS / RViz | 库本身无中间件；示例由 `automanip/examples` 自行接线（**库不依赖 autonomy / ROS / autolink**） |
| SQP·IPM QP | HPIPM | 同（`bolza/ipm` 内 `HpipmInterface`；BLASFEO=FetchContent，HPIPM=`thirdparty/hpipm`） |
| 双层 / FW | `ocs2_frank_wolfe` + `ocs2_ocs2` | `frank_wolfe` + `bilevel`（需 GLPK） |
| 切换模型 | `ocs2_switched_model_interface` | `bolza/switched_model` |
| Python | `ocs2_python_interface` / mpcnet py | **无**（已移除 pybind / 训练脚本） |
| 命名空间 | `ocs2` | `bolza` |

## 1.6　许可

- 包元数据 / 本仓库封装：Apache-2.0
- `bolza/` 算法源码：BSD-3-Clause（OCS2），见 `solver_license.txt`
- HPIPM / BLASFEO：上游 BSD-2-Clause（Gianluca Frison）
- 引用：OCS2 BibTeX + 注明 `libbolza`

## 1.7　阅读顺序

1. [02_installation.md](02_installation.md) — 依赖与编译
2. [03_getting_started.md](03_getting_started.md) — 链接库与 MPC
3. [04_robotic_examples.md](04_robotic_examples.md) §4.1 — **`Problem` / `Solve` 教程**（`bolza_tutorial_di`）
4. [04](04_robotic_examples.md) §4.3+ — 各机器人示例
5. [05](05_optimal_control_modules.md)–[07](07_solvers.md) — 建模（按需）
6. [13_faq.md](13_faq.md) — 排障
