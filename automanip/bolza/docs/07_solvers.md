# 7　求解器

- SQP / IPM 二次子问题：HPIPM（`bolza/ipm/hpipm_interface.*`，设置 `ipm::HpipmSettings`）
- 配置：[06_configuration.md](06_configuration.md)
- 运行：[04_robotic_examples.md](04_robotic_examples.md)

## 7.1　选型

| 族 | 类 | 适用 | 示例 |
|:---|:---|:---|:---|
| DDP | `GaussNewtonDDPMPC` 等 | 默认；软约束 / AL | 全部 demo |
| SQP | `SqpMpc` | 多重射击、路径约束较硬 | `legged_robot_demo … sqp` |
| IPM | `IpmMpc` | 内点不等式 | `legged_robot_demo … ipm` |
| SLP | `SlpMpc` | 低维或对照 | `ballbot_demo … slp` |

- 建议：先 DDP → 约束变硬试 SQP/IPM → 低维可试 SLP

## 7.2　代码

```cpp
#include <bolza/ddp/gauss_newton_ddp_mpc.hpp>
bolza::GaussNewtonDDPMPC mpc(mpcSettings, ddpSettings, rollout, problem, initializer);
```

```cpp
#include <bolza/sqp/sqp_mpc.hpp>
bolza::SqpMpc mpc(/* … */);
```

## 7.3　命令行

- `ballbot_demo`：`ddp` / `slp`
- `legged_robot_demo`：`ddp` / `sqp` / `ipm`
- 其余：固定 DDP
- 非法名 → 退出码 $2$

## 7.4　实时排查

- `timeHorizon` · MRT/MPC 频率
- CppAD 是否反复编译
- `nThreads`
