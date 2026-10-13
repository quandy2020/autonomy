# Display / Panel / Tool / View

## 目录布局

`autoviz/plugins/displays/` 对齐 RViz `rviz_default_plugins/displays/<module>/`：一模块一目录；跨模块工具在 `common/`；根目录仅保留框架类型（`display` / `channel_display` / `display_group` / `failed_display`）。详见 [plugins/displays/README.md](../../autoviz/plugins/displays/README.md)。

| RViz 目录 | Autoviz 目录 |
|-----------|--------------|
| `grid` / `axes` / `tf` / … | 同名 |
| `pointcloud` | `pointcloud/`（`point_cloud2_display`） |
| `point` / `twist` / `accel` / `pose_covariance` | `point/` / `twist/` / `accel/` / `pose_covariance/` |
| `temperature` | `temperature/` |
| `relative_humidity` | `relative_humidity/` |
| `fluid_pressure` | `fluid_pressure/` |
| （无） | `imu/` · `grid_map/` · `scalar_sensor/` |

## Displays

| RViz2 | Autoviz | 状态 |
|-------|---------|------|
| Grid | Grid | ✅ `plugins/displays/grid/` |
| Axes | Axes | ✅ `plugins/displays/axes/` |
| TF | TF | ✅ `plugins/displays/tf/` |
| RobotModel | RobotModel | ✅ `plugins/displays/robot_model/` + `plugins/robot/` |
| LaserScan | LaserScan | ✅ `plugins/displays/laser_scan/` |
| PointCloud2 | PointCloud2 | ✅ `plugins/displays/pointcloud/` |
| Map | Map | ✅ `plugins/displays/map/` |
| Odometry | Odometry | ✅ `plugins/displays/odometry/` |
| Path | Path | ✅ `plugins/displays/path/` |
| Marker / MarkerArray | 同名 | ✅ `marker/` · `marker_array/` |
| Pose / PoseArray | 同名 | ✅ `pose/` · `pose_array/` |
| Image | Image + Image dock | ✅ `plugins/displays/image/` |
| Camera | Camera | ✅ `plugins/displays/camera/` |
| InteractiveMarkers | InteractiveMarkers | ✅ `plugins/displays/interactive_markers/` |
| Wrench / Effort | 同名 | ✅ `wrench/` · `effort/` |
| GridCells / PointStamped / Polygon / Range | 同名 | ✅ 对应子目录 |
| PoseWithCovariance / TwistStamped / AccelStamped | 同名 | ✅ `pose_covariance/` · `screw/` + `twist/` · `accel/` |
| CameraInfo / DepthCloud | 同名 | ✅ `camera_info/` · `depth_cloud/` |
| Temperature | Temperature | ✅ `plugins/displays/temperature/` |
| RelativeHumidity | RelativeHumidity | ✅ `plugins/displays/relative_humidity/` |
| FluidPressure | FluidPressure | ✅ `plugins/displays/fluid_pressure/` |
| Imu / 其它标量传感器 | 同名 | ✅ `imu/` · `scalar_sensor/` |
| Group | Group | ✅ 根目录 `display_group` |

## 面板

| RViz2 | Autoviz | 状态 |
|-------|---------|------|
| Displays | Displays | ✅ |
| Views | Views | ✅ |
| Selection | Selection | ✅ |
| Tool Properties | 工具属性 | ✅ |
| Time | Time / Playback | ✅ `.record` |
| Transformation | Transformation | ✅ `common/transformation/` + `plugins/transformation/` |

## 工具

| RViz2 | Autoviz | 状态 |
|-------|---------|------|
| Interact / MoveCamera / FocusCamera | 同名概念 | ✅ |
| Measure | Measure | ✅ |
| Set Goal / Set Initial Pose | NavGoal / 初值 | ✅ |
| Publish Point 等 | 视实现 | ⚠️ 按面板扩展 |

## 视图

| RViz2 | Autoviz | 状态 |
|-------|---------|------|
| Orbit / FPS / TopDownOrtho / XYOrbit / ThirdPersonFollow | 同名 | ✅ `plugins/view_controllers/` |

## 相关文档

- [框架对照](framework.md) · [使用](../guide/usage.md)
