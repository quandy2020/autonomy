# Perception 感知指南

`autonomy/perception` 是 Autonomy 的**环境感知**子系统，负责将原始传感器数据（激光、点云、相机、雷达等）转化为结构化环境表示（障碍、语义、检测框等），供 `prediction`、`planning`、`map` 等模块消费。本文档为模块总入口，以下按 **§1–§9** 顺序组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§5 架构](05_architecture.md) → [§4 使用指南](04_usage.md) |
| 算法研发 | [§9 综述](09_survey.md) → [§3 数学原理](03_math.md) → [§7 视觉检测](07_vision_detection.md) |
| 集成调试 | [§2 快速开始](02_quickstart.md) → [§6 传感器管线](06_sensor_pipeline.md) → [§8 障碍感知](08_obstacle_perception.md) |

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 模块概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_math.md](03_math.md) | 数学原理 |
| 4 | [04_usage.md](04_usage.md) | 使用指南 |
| 5 | [05_architecture.md](05_architecture.md) | 模块架构设计 |
| 6 | [06_sensor_pipeline.md](06_sensor_pipeline.md) | 传感器数据管线 |
| 7 | [07_vision_detection.md](07_vision_detection.md) | 视觉检测与 ONNX 推理 |
| 8 | [08_obstacle_perception.md](08_obstacle_perception.md) | 障碍感知与代价地图 |
| 9 | [09_survey.md](09_survey.md) | 感知算法综述 |
