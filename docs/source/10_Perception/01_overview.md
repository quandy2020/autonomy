(perception-overview)=
# 1. 模块概览

感知这边现在两条实线：

1. **`perception/base`**：YOLO26 多任务 + MoGe 深度，话题在 `/perception/base/...`，见 [10](10_base.md)。
2. **`perception/follow`**：拿 track 和深度做人跟，出 GridMap 给 MPPI，见 [11](11_follow.md)。

激光 / 点云进障碍仍走 `map/costmap_2d` 的 ObstacleLayer / VoxelLayer（[08](08_obstacle_perception.md)）。推理引擎在 `common/network`（[07](07_vision_detection.md)）。

`PerceptionServer` 还是总装壳；探索相关配置在仓库里，但后端没编进当前构建（[12](12_exploration.md)）。

```
Camera → base → follow → /perception/follow/grid → MPPI
Laser  → ObstacleLayer → costmap → Planning
```
