# 11. Follow

目录：`autonomy/perception/follow/`。订阅 base 的 tracks 和 MoGe 深度，维护一块 2.5D `grid_map`，话题 `/perception/follow/grid` 给 MPPI 的 GridMap critic 用；**不**往 OccupancyGrid costmap 里灌。

```
/perception/base/tracks ─┐
/perception/base/depth  ─┼─> follow ─> target / path / grid
CameraInfo + Odom + TF  ─┘              │
                                        ▼
                              ControllerServer → MPPI
```

| 话题 | 含义 |
|------|------|
| `/perception/follow/select` | 指定 track id；空则取最大行人框 |
| `/perception/follow/target` | map 系目标位姿 |
| `/perception/follow/path` | 局部跟随路径 |
| `/perception/follow/grid` | 层：`elevation` / `obstacle` / `traversability` |

主要文件：`localizer`、`planner`、`grid`、`follow_component`。启动走 follow 的 DAG / launch，控制侧打开 MPPI 的 GridMap critic。说明以源码 `follow/README.md` 为准。
