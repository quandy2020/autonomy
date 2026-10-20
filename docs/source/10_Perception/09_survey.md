# 9. 感知综述

选型或评审前，快速对齐「能交付什么」。

汇总可上线能力与壳/占位，避免按路线图验收。

| 需求 | 路径 |
|------|------|
| 避障导航 | costmap |
| 检测/深度话题 | base |
| 跟人 | base + follow + MPPI |
| 主动探索 | 勿依赖仓库探索配置 |

### 能当真用的

| 能力 | 位置 | 文档 |
|------|------|------|
| YOLO26 多任务 + MoGe | `perception/base` | [§10](10_base.md) |
| 行人跟随 GridMap | `perception/follow` | [§11](11_follow.md) |
| 激光/点云 costmap | `map/costmap_2d` | [§8](08_obstacle_perception.md) |
| ONNX/TensorRT 引擎 | `common/network` | [§7](07_vision_detection.md) |

### 未完成 / 壳

| 项 | 说明 |
|----|------|
| `PerceptionServer` 总装 | 仅 Start/Shutdown、node；不接 Base/Follow |
| RGB-D 探索后端 | 开关打开只 WARN「not built」 |
| 统一 `perception.lua` 驱动 YOLO | 未接主进程 |

### 常见坑

| 现象 | 说明 |
|------|------|
| exploration.pb.txt 无地图 | 预期 |
| 找 GetDetections API | 不存在 |

### 延伸阅读

[§5 架构](05_architecture.md) · [§12 探索](12_exploration.md)
