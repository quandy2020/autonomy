# 11. Follow

跟人走、配 MPPI GridMap critic，或订阅 `/perception/follow/*`。

`autonomy/perception/follow/`：订阅 base 的 **tracks + MoGe depth**，维护 2.5D `grid_map`，话题 `/perception/follow/grid` 给 MPPI；**不写** OccupancyGrid costmap。

```text
/perception/base/tracks ─┐
/perception/base/depth  ─┼─> follow ─> target / path / grid
CameraInfo + Odom + TF  ─┘              └─> ControllerServer → MPPI
```

| 话题 | 含义 |
|------|------|
| `/perception/follow/select` | 指定 track id；空则最大行人框 |
| `/perception/follow/target` | map 系目标 |
| `/perception/follow/path` | 局部路径 |
| `/perception/follow/grid` | elevation / obstacle / traversability |

- 必须先有 base tracks/depth。  
- 与激光 costmap **并行、不替代**（[§8](08_obstacle_perception.md)）。  
- 组件：`localizer`、`planner`、`grid`、`follow_component`；launch 见 follow DAG。

### 常见坑

| 现象 | 先查 |
|------|------|
| 无 grid | tracks/depth 话题、TF/Odom |
| MPPI 不吃障 | GridMap critic 是否启用 |

说明：`follow/README.md`。
