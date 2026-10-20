(perception-overview)=
# 1. 模块概览

需要一张「感知在仓库里到底分几条线」的总图。

Perception 不是单进程全能 Server，而是 **三条可独立部署的路径**：

1. **视觉 Base**（`perception/base`）— 多任务 YOLO26 + MoGe 深度，话题 `/perception/base/...` → [§10](10_base.md)  
2. **Follow**（`perception/follow`）— tracks + depth → GridMap → MPPI → [§11](11_follow.md)  
3. **激光/点云 → costmap** — ObstacleLayer / VoxelLayer → Planning → [§8](08_obstacle_perception.md)  

推理引擎在 `common/network`（[§7](07_vision_detection.md)）。`PerceptionServer` 仅进程壳 + 探索开关告警（[§12](12_exploration.md) 后端未编）。

- Base/Follow 用 **DAG/launch** 起，不依赖 `PerceptionServer` 里的插件表。  
- 导航默认可**只开 costmap**，零视觉。  
- 探索相关 Lua **不能**当已上线能力验收。

| 目标 | 顺序 |
|------|------|
| 先跑通导航 | costmap + `/scan` |
| 加视觉 | Base 出 detection/track → 再 Follow |
| 探索建图 | 等后端或自研，勿靠 enable_rgbd_exploration |

### 数据流

```text
Camera → base → follow → /perception/follow/grid → MPPI
Laser  → ObstacleLayer → costmap → Planning
```

### 常见坑

| 误解 | 事实 |
|------|------|
| 一个 Server 管所有感知 | 否 |
| 检测结果默认进 costmap | 否（Follow 走 GridMap） |

### 延伸阅读

[00 指南](00_guide.md) · [§9 综述](09_survey.md)
