# API 索引

模块说明与主文档合一，避免双份维护。按目录读头文件，或跳转：

| 源码 | 文档 |
|------|------|
| `argus/odometry/` | [02 架构](../02_architecture.md)、[09 配置](../09_config.md)、[10 用法](../10_usage.md) |
| `argus/frontend/` | [05 前端与融合](../05_frontend_fusion.md) |
| `argus/backend/` | [02 架构](../02_architecture.md)、[04 数学](../04_math.md)、[07 回环](../07_loop_closure.md) |
| `argus/fusion/` / `perception/` | [05 前端与融合](../05_frontend_fusion.md)、[04 数学](../04_math.md) |
| `argus/map/` / `mapping/` | [06 地图](../06_map.md) |
| `argus/loop_closure/` | [07 回环](../07_loop_closure.md) |
| `argus/relocalization/` | [08 重定位](../08_relocalization.md)、[03 时序](../03_sequence.md) |
| `argus/sensor/` / `common/` | 头文件即 API |

消费侧：`find_package(Argus)` → `Argus::argus`（`Argus::odometry` 等为同库别名）。
