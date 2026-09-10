# Follow — person following closed loop

Follow sits on top of `base` tracks and depth:

```text
/perception/base/tracks ─┐
/perception/base/depth  ─┼─> follow ─> /perception/follow/target
CameraInfo + Odom + TF  ─┘           /perception/follow/path
                                     /perception/follow/grid
```

| Topic | Type | Role |
| --- | --- | --- |
| `/perception/follow/select` | `std_msgs/String` | Input track id; empty = largest person box |
| `/perception/follow/target` | `PoseStamped` | Locked person in `map` |
| `/perception/follow/path` | `Path` | Local follow path in `map` |
| `/perception/follow/grid` | `GridMap` | Rolling 2.5D safety map |

Depth for the grid should come from MoGe via `base` (`DEPTH_BACKEND_MOGE`).
`task/TrackingClient` subscribes to these follow topics.
