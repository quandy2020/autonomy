# 4. 局部控制器

实现目录 `autonomy/control/controller/`，经 `controller_plugins` 加载。FollowPath 周期调用 `ComputeVelocityCommands`；选型详见 [00 指南](00_guide.md)、[05 综述](05_survey.md)。

按「问题 → 算法族 → 插件 id → conf 字段」阅读；默认项以代码注册为准。

| 插件 | 目录 | 适合 | 文档 |
|------|------|------|------|
| Graceful | `graceful_controller/` | 平滑差速跟踪、算力低 | [01](controller/01_graceful_controller.md) |
| MPPI | `mppi_controller/` | 动态障、多 critic、Follow GridMap | [02](controller/02_mppi_controller.md) |
| RPP | `pure_pursuit_controller/` | 默认室内、调参少 | [03](controller/03_rpp_controller.md) |
| NMPC | `nmpc_controller/` | 显式时域约束、OCP | [04](controller/04_nmpc_controller.md) |
| NeuPAN | `neupan_controller/` | 激光 + ONNX 距离场 | [05](controller/05_neupan_controller.md) |
| SANDO | `sando_controller/` | 走廊 + MIQP、动态障碍 | [06](controller/06_sando_controller.md) |
| Policy | `policy_controller/` | 自训 ONNX/TRT 策略 | [07](controller/07_policy_controller.md) |

**没有** DWB、TEB 插件。

## 怎么选（简表）

| 情况 | 控制器 |
|------|--------|
| 先跑通跟线 | RPP 或 Graceful |
| 行人/动态障 | MPPI 或 NeuPAN |
| 硬约束、显式 OCP | NMPC |
| 时空走廊、动态盒 | SANDO |
| 已有 RL/IL 策略图 | Policy |

NeuPAN / Policy 需准备 ONNX（或 TRT）与路径 sidecar。Checker：[02](02_checkers.md)。输出速度平滑库（未默认接入）：[03](03_velocity_smoother.md)。

## 共性依赖

有效 TF、`map`/`base_link`、多数插件要 costmap（Policy 内部加锁策略不同）。「有 Path 无 cmd_vel」先查插件是否加载、`follow_path_active_`、里程计与 costmap current。
