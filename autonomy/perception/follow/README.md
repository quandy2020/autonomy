# Follow — person following closed loop

Follow sits on top of `base` tracks and MoGe depth, builds a rolling
`grid_map` 2.5D LocalGrid, and drives MPPI via `/perception/follow/grid`
(not OccupancyGrid / costmap injection).

```text
/perception/base/tracks ─┐
/perception/base/depth  ─┼─> follow ─> /perception/follow/target
CameraInfo + Odom + TF  ─┘           /perception/follow/path
                                     /perception/follow/grid  (GridMap 2.5D)
                                              │
                                              ▼
                                    ControllerServer GridMapBuffer
                                              │
                                              ▼
                                    MPPI GridObstaclesCritic
```

| Topic | Type | Role |
| --- | --- | --- |
| `/perception/follow/select` | `std_msgs/String` | Input track id; empty = largest person box |
| `/perception/follow/target` | `PoseStamped` | Locked person in `map` |
| `/perception/follow/path` | `Path` | Local follow path in `map` |
| `/perception/follow/grid` | `GridMap` | Layers: `elevation`, `obstacle`, `traversability` |

## Layout / launch

```text
perception/
  conf/base.pb.txt          # GetProtoConfig for base
  conf/follow.pb.txt        # GetProtoConfig for follow
  base/dag/base.dag
  follow/dag/follow.dag
  launch/perception.launch
```

```bash
export AUTOLINK_DAG_PATH=$PWD/autonomy:$AUTOLINK_DAG_PATH
export AUTOLINK_CONF_PATH=$PWD/autonomy/perception:$AUTOLINK_CONF_PATH
export AUTOLINK_LIB_PATH=/workspace/autonomy/build/autonomy/lib:$AUTOLINK_LIB_PATH
autolink launch start perception.launch
```

Depth should come from MoGe via `base` (`DEPTH_BACKEND_MOGE`).
Tracker BT uses `controller_id=mppi_controller` by default.
