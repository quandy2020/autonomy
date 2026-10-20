# 4. 局部控制器

实现都在 `autonomy/control/controller/`，下面按目录链专题页。

| 插件 | 目录 | 文档 |
|------|------|------|
| Graceful | `graceful_controller/` | [01](controller/01_graceful_controller.md) |
| MPPI | `mppi_controller/` | [02](controller/02_mppi_controller.md) |
| RPP | `pure_pursuit_controller/` | [03](controller/03_rpp_controller.md) |
| NMPC | `nmpc_controller/` | [04](controller/04_nmpc_controller.md) |
| NeuPAN | `neupan_controller/` | [05](controller/05_neupan_controller.md) |
| SANDO | `sando_controller/` | [06](controller/06_sando_controller.md) |
| Policy | `policy_controller/` | [07](controller/07_policy_controller.md) |

RPP / Graceful 算力低；MPPI、SANDO 更重。NeuPAN / Policy 要 ONNX（或 TRT）模型。Checker 与 Smoother 见 [02](02_checkers.md)、[03](03_velocity_smoother.md)。
