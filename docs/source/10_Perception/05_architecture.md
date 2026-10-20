# 5. Perception 模块架构

### 5.1 实现状态

| 组件 | 状态 |
|------|------|
| `perception/base`（YOLO26 任务 + MoGe） | — |
| `perception/follow` | — |
| `common/network`、`vision_msgs` | — |
| `ObstacleLayer` / `VoxelLayer` | — |
| `PerceptionServer` 总装 | — |

### 5.2 分层

| 层 | 组件 | 职责 |
|----|------|------|
| 跟随 | `follow/` | tracks + depth → GridMap → MPPI |
| 视觉 | `base/` | 检测 / 跟踪 / 深度等任务 |
| 推理 | `common/network` | ONNX / TensorRT |
| 地图障碍 | `map/costmap_2d` | 激光/点云 → costmap |
| 总装 | `PerceptionServer` | 进程入口（接线中） |

### 5.3 数据流

```
Camera → base → /perception/base/{detections,tracks,depth}
→ follow → /perception/follow/grid → Controller (MPPI)
Laser/PointCloud → ObstacleLayer → costmap → Planning
```
