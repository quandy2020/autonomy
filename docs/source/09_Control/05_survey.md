# 5. 控制综述

Planning 出几何 Path（大约 1–5 Hz）；Control 跟路径出 `cmd_vel`（大约 10–50 Hz），costmap 更多用局部或共享副本。

仓库里已经有的控制器：

- 几何：RPP（`pure_pursuit_controller/`）、Graceful（`graceful_controller/`）
- 采样：MPPI（`mppi_controller/`）
- OCP：NMPC（`nmpc_controller/`，C-GMRES / DDP / FMPC）
- 学习距离场：NeuPAN（`neupan_controller/`）
- 走廊 MIQP：SANDO（`sando_controller/`）
- 通用策略图：Policy（`policy_controller/`）

没有 DWB、TEB，也不准备单独写页。室内默认 RPP + SimpleGoal；动态障碍 MPPI / NeuPAN；硬约束 NMPC / SANDO；自训策略走 Policy。Checker / Smoother 见 [02](02_checkers.md)、[03](03_velocity_smoother.md)。
