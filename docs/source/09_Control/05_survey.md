# 5. 控制综述

Planning 约 1–5 Hz 给几何 **Path**；Control 约 10–50 Hz 跟路径发 **cmd_vel**，并用局部或共享 **Costmap2D** 做碰撞/regulated 约束。

## 已有控制器（源码 + 插件注册）

| 类型 | 插件 | 目录 |
|------|------|------|
| 几何跟踪 | RPP | `pure_pursuit_controller/` |
| 几何跟踪 | Graceful | `graceful_controller/` |
| 采样 MPC | MPPI | `mppi_controller/` |
| OCP | NMPC（C-GMRES / DDP / FMPC） | `nmpc_controller/` |
| 学习距离场 | NeuPAN | `neupan_controller/` |
| 走廊 MIQP | SANDO | `sando_controller/` |
| 通用策略图 | Policy | `policy_controller/` |

无 DWB、TEB。

## 怎么选

- **室内默认**：RPP 或 Graceful + SimpleGoal / SimpleProgress（Server 默认 checker）。
- **动态障碍 / 跟人**：MPPI（开 Grid critic 接 Follow）或 NeuPAN（激光 + ONNX）。
- **显式预测约束**：NMPC。
- **动态盒 + 走廊**：SANDO（算力与调参成本更高）。
- **自研网络**：Policy + `policy_config` sidecar + 安全滤波。

## Checker / 平滑

- Goal/Progress：默认仅 Simple 两种；Stopped / Pose / Position 有类未进 Server，见 [02](02_checkers.md)。
- **VelocitySmoother** 库在 `utils/`，**未** 接入 FollowPath；`OdomSmoother` 只平滑订阅的里程计。

## 与 Planning 边界

Planning 不输出速度；到点、进度、失败码在 Control + checker。路径贴障时先查 map 膨胀，再调控制器 regulated / critic 权重。

专题：[04 算法总览](04_controller_algorithms.md)、[01 架构](01_architecture.md)。
