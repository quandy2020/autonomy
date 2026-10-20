# 5. Perception 模块架构

画数据流或拆任务，需要分清「视觉组件」和「地图障碍层」谁依赖谁。

说明仓库里感知相关代码**实际落在哪些目录**，以及 `PerceptionServer` 在总装里的位置（壳 + 探索占位）。

1. **Base / Follow** 走 Autolink 组件与 DAG，不经过 Server 里的插件表。  
2. **ObstacleLayer / VoxelLayer** 在 `map/costmap_2d`，输入激光/点云，输出 costmap 给 Planning。  
3. **探索** 只有配置与 WARN，无后端实现（[§12](12_exploration.md)）。

| 问题 | 去看 |
|------|------|
| 检测话题名 | [§10 Base](10_base.md) |
| MPPI 用的栅格 | [§11 Follow](11_follow.md) |
| 导航避障 | [§8](08_obstacle_perception.md) |
| ONNX 细节 | [§7](07_vision_detection.md) |

### 分层（按代码目录）

| 层 | 路径 | 职责 |
|----|------|------|
| 跟随 | `perception/follow/` | tracks + depth → GridMap → MPPI |
| 视觉任务 | `perception/base/` | YOLO26 多任务 + MoGe 深度 |
| 推理 | `common/network/` | ONNX / TensorRT |
| 地图障碍 | `map/costmap_2d/layers/` | 激光/点云 → costmap |
| 进程壳 | `perception_server.*` | node 生命周期；探索开关告警 |

### 数据流

```text
Camera → base → /perception/base/{detections,tracks,depth}
              → follow → /perception/follow/grid → Controller (MPPI)

Laser / PointCloud → ObstacleLayer / VoxelLayer → costmap → Planning
```

### 常见坑

| 误解 | 事实 |
|------|------|
| 「一个 PerceptionServer 管所有感知」 | Base/Follow 独立部署；Server 未串任务 |
| 「探索 Lua 能跑 TARE」 | 后端未编入 |
| 「视觉结果进 costmap」 | Follow 用 GridMap；激光才进 costmap |

### 延伸阅读

[§9 综述](09_survey.md) · [§12 探索](12_exploration.md)
