# Display modules (RViz-aligned layout)

Layout mirrors `rviz_default_plugins/displays/<module>/`: one directory per
display family. Cross-cutting helpers live in `common/`. Framework types stay
at this root.

## Root (framework)

| File | Role |
|------|------|
| `display.hpp` / `display.cpp` | Base `Display` |
| `display_group.*` | Nested display group |
| `channel_display.hpp` | Channel-backed display template |
| `failed_display.*` | Placeholder for load failures |

## Modules (RViz names)

`accel/`, `axes/`, `camera/`, `camera_info/`, `depth_cloud/`, `effort/`,
`grid/`, `grid_cells/`, `image/`, `interactive_markers/`, `laser_scan/`,
`map/`, `marker/`, `marker_array/`, `odometry/`, `path/`, `point/`,
`pointcloud/`, `polygon/`, `pose/`, `pose_array/`, `pose_covariance/`,
`fluid_pressure/`, `range/`, `relative_humidity/`, `robot_model/`,
`screw/` (shared by `twist/` · `accel/`), `temperature/`, `tf/`, `twist/`,
`wrench/`

## Autoviz-only

| Directory | Displays |
|-----------|----------|
| `imu/` | Imu |
| `grid_map/` | GridMap |
| `scalar_sensor/` | Illuminance |

## Shared helpers

`../transformation/` (under `plugins/transformation/`, not under `displays/`) —
RViz-aligned `TFWrapper` / `TFFrameTransformer` / `TransformerGuard` over
Autoviz `transform::Buffer` (no ROS).

`common/` — `transform_utils`, `proto_payload_utils`, `obj_mesh`,
`ogre_*_draw`, `urdf_model`, `scalar_sensor_utils`, etc.

`../robot/` (under `plugins/robot/`, not under `displays/`) — RViz-aligned
`robot::Robot` / `RobotLink` / `RobotJoint` / `LinkUpdater` / `TFLinkUpdater`
(used by `robot_model/`).

`../view_controllers/` (under `plugins/view_controllers/`, not under `displays/`) —
RViz-aligned Orbit / XYOrbit / TopDownOrtho / FPS / ThirdPersonFollow / Frame
wrappers over `rendering::ViewController` (no ROS).

## Includes

Use modular paths (no root-level compatibility shims):

```cpp
#include "autoviz/plugins/displays/pointcloud/point_cloud2_display.hpp"
```

Registration lives in `autoviz/common/display_registry.cpp`.
