# 0. Perception 指南

## 背景

开了「感知 Server」却没有任何输出、Follow 无 grid、探索配置不生效——需要分清 **哪条数据路径真在跑**：`PerceptionServer` 仍是总装壳，探索后端 **未编入**（[12 Exploration](12_exploration.md)）。推理引擎在 `common/network`；激光避障仍走 costmap 老路径。

## 思路

现在能当真用的能力：

| 能力 | 说明 | 文档 |
|------|------|------|
| **Base** | YOLO26 多任务 + MoGe 深度 | [10 Base](10_base.md) |
| **Follow** | tracks + 深度 → GridMap → MPPI | [11 Follow](11_follow.md) |
| **Costmap 障碍** | 激光/点云 → Obstacle/VoxelLayer | [08 障碍感知](08_obstacle_perception.md) |
| **network** | ONNX / TensorRT 引擎 API | [07 视觉检测](07_vision_detection.md) |

```text
Camera → base → follow → /perception/follow/grid → MPPI
Laser  → ObstacleLayer → costmap → Planning
```

- **先分清三条路径** — 视觉 Base、Follow、激光 costmap，不要混成一个 Server。  
- **权重与 engine 不进 git** — 开任务前确认 `config/perception/base/` 下模型已导出。  
- **Follow 不写 OccupancyGrid** — 走 GridMap + MPPI critic。


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 概览](01_overview.md) | 现状与数据流 |
| [02 快速开始](02_quickstart.md) | 最短起感知相关节点 |
| [03 数学](03_math.md) | 模型与几何 |
| [04 用法](04_usage.md) | 配置与话题 |
| [05 架构](05_architecture.md) | 模块边界 |
| [06 传感器管线](06_sensor_pipeline.md) | 数据怎么进模块 |
| [09 综述](09_survey.md) | 能力边界 |
| [12 探索](12_exploration.md) | 配置存在、后端未构建 |

**检测/跟踪/深度话题** → [10 Base](10_base.md)。  
**跟人走** → [11 Follow](11_follow.md) + [MPPI](../09_Control/controller/02_mppi_controller.md)。  
**只要避障图层** → [08](08_obstacle_perception.md)。

### 2. 动手时记住

1. 探索相关 Lua/开关在仓库里，别按「能探索」去验收整车。  
2. 预测整链未闭合时，不要在本章找 prediction 话题。

### 3. 常见坑

| 现象 | 先查 |
|------|------|
| 无检测输出 | 模型路径；任务 kind；相机话题 |
| Follow 无 grid | base tracks/depth；TF/Odom |
| 开了 exploration 无效果 | 后端未构建，属预期 |
