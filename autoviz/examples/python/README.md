# Autoviz Python Examples

Small tutorials that publish [`automsgs`](../../automsgs) messages over
[`autolink`](../../autolink) for live Autoviz verification.

**C++ equivalents:** see [`../cpp`](../cpp) (`autoviz_cpp_*` binaries, same channels).

## Prerequisites

1. Build the autonomy workspace (`colcon build` — includes `automsgs` and
   **autolink Python** by default).

   Python module path: `build/autonomy/python/`

   If you configured before this default, re-run once:

```bash
colcon build --cmake-args -DAUTOLINK_BUILD_PYTHON=ON
```

   Fallback standalone build:

```bash
./src/autonomy/autoviz/examples/python/setup_autolink_python.sh
```

2. Install protobuf if needed:

```bash
/usr/bin/python3 -m pip install protobuf
```

**Isaac Sim / Docker note:** `.bashrc` often aliases `python3` to Isaac Sim's
`python.sh`, which causes `SRE module mismatch`. Either:

```bash
./examples/python/01_tutorial_poses.py
# or
/usr/bin/python3 examples/python/01_tutorial_poses.py
```

**macOS:** `_bootstrap` sets `DYLD_LIBRARY_PATH`. Prefer system/Homebrew Python
(`/usr/bin/python3` or `brew --prefix python@3.12`). See
[`../../deploy/macos/README.md`](../../deploy/macos/README.md).

Bare `python3 ...` is also handled via auto re-exec when possible.

Optional overrides:

```bash
export AUTONOMY_BUILD_DIR=/path/to/build/autonomy
export AUTOLINK_PYTHON_DIR=/path/to/build/autolink-python
export AUTOVIZ_PYTHON=/opt/venv/bin/python3
```

## Coverage matrix

| Autoviz piece | Tutorial | How to verify |
|---|---|---|
| Odometry / Path / Pose* | 01 | 3D Displays |
| OccupancyGrid Map | 02 | Map Display |
| Image | 03 | Image panel / Display |
| Imu / Scan / PC / Range / Depth+Info | 04 | Displays; **DepthCloud**=`/fake/depth`+`camera_info`; **Camera**=`/fake/image`+info |
| TF / TfTree | 05 | TF Display + Transform Tree panel |
| RobotModel | 06 | RobotModel Display |
| Marker / MarkerArray / Interactive | 07 | Displays + Interact tool |
| Effort / Wrench / Temperature | 08 | Displays |
| Plot | 09 | Plot panel |
| Log | 10 | Log panel (`/fake/log`) |
| State Transitions | 11 | State Transitions panel |
| Channel Graph | 12 | Channel Graph panel |
| Raw Messages | 13 | Raw Messages panel |
| Table | 14 | Table panel |
| Diagnostics (via Table) | 15 | Table on `/diagnostics` |
| Parameters | 16 | Parameters panel |
| Vision annotations | 17 | Image Annotations |
| Home-service all topics | 18 | Multi-panel smoke |
| GridCells / Point / Polygon / Twist / Accel | 19 | 3D Displays |
| Illuminance / Pressure / Humidity + Gauge | 20 | Displays + Gauge / Indicator (`/fake/gauge` · `data`) |
| Audio | 21 | Audio panel |
| Service Call | 22 | Service Call → `/fake/echo` |
| Map (GPS) | 23 | Map panel · `/gps/fix` |
| Teleop | 24 | Teleop panel + this echo script |

**UI-only / no publisher script:** Publish, Variables, Variable Slider, Channels browser,
Problems, Help, Playback (needs `.record`), Strata\* Displays, Tools (2D Nav Goal /
Pose Estimate / Publish Point — exercise manually against 01/18).

## 01 — Poses (Odometry / Path / Pose / PoseArray / PoseWithCovariance)

Publishes pose-family messages on Autoviz default channels (Fixed Frame=`map`):

| Display | Channel | Type |
|---|---|---|
| Odometry | `/fake/odom` | `nav_msgs/Odometry` |
| Path | `/fake/path` | `nav_msgs/Path` |
| Pose | `/fake/pose` | `geometry_msgs/PoseStamped` |
| PoseArray | `/fake/pose_array` | `geometry_msgs/PoseArray` |
| PoseWithCovariance | `/fake/pose_with_covariance` | `geometry_msgs/PoseWithCovarianceStamped` |

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/01_tutorial_poses.py
./build/autonomy/bin/autoviz -c config/default.autoviz
```

Enable the corresponding Displays. Path / Odometry are on by default in
`config/default.autoviz`; add Pose / PoseArray / PoseWithCovariance from the catalog.

## 02 — Occupancy Grid

Publishes `automsgs.msgs.map_msgs.OccupancyGrid` on `/fake/occupancy_grid`
(default Autoviz Map display). Fixed Frame should be `map`.

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/02_tutorial_occupancy_grid.py

./build/autonomy/bin/autolink channel echo /fake/occupancy_grid --once
./build/autonomy/bin/autoviz -c config/default.autoviz
```

## 03 — Image

Publishes `automsgs.msgs.sensor_msgs.Image` (`rgb8`) on `/fake/image`
(default Autoviz Image / Camera channel).

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/03_tutorial_image.py

./build/autonomy/bin/autolink channel echo /fake/image --once
./build/autonomy/bin/autoviz
```

## 04 — Sensors (all-in-one)

Publishes common `sensor_msgs` on Autoviz default `/fake/*` channels:

| Sensor | Channel | Type |
|---|---|---|
| IMU | `/fake/imu` | `Imu` |
| Camera RGB | `/fake/image` | `Image` (rgb8) |
| CameraInfo | `/fake/camera_info` | `CameraInfo` |
| Camera RGBD | `/fake/depth` | `Image` (16UC1 mm) + CameraInfo |
| Lidar 2D | `/fake/scan` | `LaserScan` |
| Lidar 3D | `/fake/point_cloud` | `PointCloud` |
| Lidar 3D | `/fake/point_cloud2` | `PointCloud2` |
| Range | `/fake/range` | `Range` |

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/04_tutorial_sensor.py
./build/autonomy/bin/autoviz
```

In Autoviz: enable Imu / Image / DepthCloud / LaserScan / PointCloud /
PointCloud2 / Range displays (or add from the display catalog).

**DepthCloud:** channel=`/fake/depth`, property `camera_info_channel`=`/fake/camera_info`.  
**Camera Display:** image=`/fake/image` + CameraInfo=`/fake/camera_info` (frustum / projection).

## 05 — Transform Tree

Publishes `automsgs.msgs.tf2_msgs.TFMessage` on `/tf` (default TF display).
Fixed Frame should be `map`.

```text
map → base_link → laser
                → camera
```

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/05_tutorial_transform_tree.py
./build/autonomy/bin/autoviz -c config/default.autoviz
```

## 06 — URDF / RobotModel (PR2)

Publishes a simplified **PR2** (box/cylinder geometry, no external meshes):

| Channel | Type |
|---|---|
| `/robot_description` | `std_msgs/String` (URDF XML) |
| `/joint_states` | `sensor_msgs/JointState` |
| `/tf` | `tf2_msgs/TFMessage` (`map` → `base_link`) |

URDF file: `examples/python/urdf/pr2_simple.urdf`

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/06_tutorial_urdf.py
./build/autonomy/bin/autoviz
```

Autoviz RobotModel: Enabled, Channel=`/joint_states`, Description Topic=`/robot_description`,
Root Link=`base_link`, Fixed Frame=`map`. Clear `urdf_path` if it still points at TurtleBot.

## 07 — Visualization Markers

Publishes Marker / MarkerArray / InteractiveMarker:

| Display | Channel | Type |
|---|---|---|
| Marker | `/fake/marker` | `visualization_msgs/Marker` (moving arrow) |
| MarkerArray | `/fake/marker_array` | cube / sphere / cylinder / line / text |
| InteractiveMarkers | `/fake/update` (+ init `/fake/init`) | draggable `MOVE_3D` cube |

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/07_tutorial_visualizarion_marker.py
./build/autonomy/bin/autoviz
```

Enable Marker / MarkerArray / InteractiveMarkers. Use the **Interact** tool to
drag the yellow interactive cube (feedback on `/fake/feedback`).

## 08 — Effort / Wrench / Temperature

| Display | Channel | Type |
|---|---|---|
| Effort (+ RobotModel) | `/joint_states` (+ `/robot_description`, `/tf`) | `sensor_msgs/JointState` with `effort` |
| Wrench | `/fake/wrench` | `geometry_msgs/WrenchStamped` |
| Temperature | `/fake/temperature` | `sensor_msgs/Temperature` |

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/08_tutorial_force_joint.py
./build/autonomy/bin/autoviz
```

Fixed Frame=`map`. Enable Effort (clear `urdf_path`, Description=`/robot_description`),
Wrench, Temperature. Optional: RobotModel for the PR2 mesh.

## 09 — Plot

Publishes numeric streams for the Plot panel (time-series):

| Channel | Type | Field path |
|---|---|---|
| `/fake/plot/sine` | `std_msgs/Float64` | `data` |
| `/fake/plot/cosine` | `std_msgs/Float64` | `data` |
| `/fake/plot/cmd` | `geometry_msgs/Twist` | `linear.x` / `angular.z` |

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/09_tutorial_plot.py
./build/autonomy/bin/autoviz
```

Open Plot panel → Add series → set Channel + Field path as above.

## 10 — Log

Publishes `foxglove.Log` JSON on `/fake/log` (DEBUG/INFO/WARN/ERROR × namespaces
`sensor` / `planner` / `controller` / `autolink`) for the Log panel.

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/10_tutorial_log.py
./build/autonomy/bin/autoviz
```

Log panel settings: **Topic** = `/fake/log`. Filter by min level, namespace, or
search. **Capture glog** only shows Autoviz's own process glog (not this script).

## 11 — State Transitions

Publishes discrete string states for the State Transitions timeline:

| Channel | Type | Field | Example states |
|---|---|---|---|
| `/fake/state/nav` | `std_msgs/String` | `data` | IDLE → PLANNING → NAVIGATING → RECOVERY |
| `/fake/state/ctrl` | `std_msgs/String` | `data` | IDLE → TRACKING → HOLD |
| `/fake/state/mode` | `std_msgs/String` | `data` | MANUAL → AUTO → ESTOP |

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/11_tutorial_state_transitions.py
./build/autonomy/bin/autoviz
```

Open State Transitions → Add series (or drag from Channels) → Channel + path `data`.

## 12 — Channel Graph

Spins a multi-node Autolink topology for the Channel Graph panel (discovery graph):

```
/demo/sensor ──► /fake/graph/scan|imu ──► /demo/perception
                                           │
                                           ▼
                                    /fake/graph/objects
                                           │
                                           ▼
/demo/map_server ◄── /fake/graph/get_map ◄── /demo/planner
                                           │
                                           ▼
                                    /fake/graph/path
                                           │
                                           ▼
                                    /demo/controller → /fake/graph/cmd
```

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/12_tutorial_channel_grpah.py
./build/autonomy/bin/autoviz
```

Open Channel Graph → enable Channels/Services → Group=`/fake/graph` (optional).

## 13 — Raw Messages

Publishes nested protobufs for the Raw Messages inspector (`DebugString` + optional path):

| Channel | Type | Sample Path |
|---|---|---|
| `/fake/raw/odom` | `nav_msgs/Odometry` | `pose.pose.pose.position.x` |
| `/fake/raw/poses` | `geometry_msgs/PoseArray` | `poses[0].position.y` |
| `/fake/raw/imu` | `sensor_msgs/Imu` | `linear_acceleration.z` |

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/13_tutorial_raw_messages.py
./build/autonomy/bin/autoviz
```

Open Raw Messages → select a channel (empty Path = full message).

## 14 — Table

Publishes repeated protobuf arrays for the Table panel (one row per element):

| Channel | Type | Array path | Columns |
|---|---|---|---|
| `/fake/table/status` | `diagnostic_msgs/DiagnosticArray` | `status` | level / name / message / hardware_id |
| `/fake/table/poses` | `geometry_msgs/PoseArray` | `poses` | position / orientation |

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/14_tutiorial_table.py
./build/autonomy/bin/autoviz
```

Open Table → Channel + Array path (or drop a repeated field from Channels).

## 15 — Diagnostics

Publishes `diagnostic_msgs/DiagnosticArray` on `/diagnostics` (OK / WARN / ERROR / STALE):

| name | hardware_id | Notes |
|---|---|---|
| `power:battery` | `batt0` | voltage / soc |
| `system:cpu` | `cpu0` | load / temp |
| `sensor:lidar` | `lidar0` | rate / dropouts |
| `sensor:gps` | `gps0` | satellites / fix |
| `sensor:camera` | `cam0` | periodically STALE |
| `nav:controller` | `ctrl0` | intermittent ERROR |

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/15_tutorial_diagnostics.py
./build/autonomy/bin/autoviz
```

No dedicated Diagnostics panel yet — use **Table**: Channel=`/diagnostics`,
Array path=`status` (sort/filter by `level` / `name`). Or Raw Messages for full KeyValue.

## 16 — Parameters

Hosts an Autolink `ParameterServer` on node `/autoviz/params`:

| Name | Type | Default |
|---|---|---|
| `max_speed` | double | `1.5` |
| `goal_tol` | double | `0.05` |
| `enable_estop` | bool | `false` |
| `waypoint_count` | int | `8` |
| `robot_name` | string | `demo_bot` |

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/16_tutorial_parameters.py
./build/autonomy/bin/autoviz
```

Open Parameters → Node=`/autoviz/params` → edit Value (double-click). Filter by name.

## 17 — Vision (`vision_msgs`)

Publishes camera image + 2D/3D vision detections:

| Channel | Type | View |
|---|---|---|
| `/fake/vision/image` | `sensor_msgs/Image` | Image panel |
| `/fake/vision/detections2d` | `vision_msgs/Detection2DArray` | Image → Annotations |
| `/fake/vision/boxes2d` | `vision_msgs/BoundingBox2DArray` | Image → Annotations |
| `/fake/vision/detections3d` | `vision_msgs/Detection3DArray` | Table (`detections`) / Raw Messages |

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/17_tutorial_visions.py
./build/autonomy/bin/autoviz
```

Image panel: Channel=`/fake/vision/image`, enable annotation
`/fake/vision/detections2d` (or `boxes2d`).

## 18 — Robot all-data sim (home service)

One-process publisher covering the home-service debug topic map
(sensors → localization → nav → perception → control → diagnostics → task → debug).

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/18_tutorial_robot_all_data_sim.py
./build/autonomy/bin/autoviz
```

| Category | Channels (highlights) |
|---|---|
| Sensors | `/camera/image_raw`, `/camera/depth/image_raw`, `/scan`, `/points`, `/imu/data`, `/gps/fix`, `/battery_state`, `/ultrasonic/distance`, `/joint_states`, `/wrench` |
| Localization | `/odom`, `/tf`, `/map`, `/amcl_pose`, `/robot_description` |
| Navigation | `/cmd_vel`, `/plan`, `/local_plan`, `/move_base/status`, `/waypoint` |
| Perception | `/detected_objects`, `/people`, `/semantic_map` |
| Control | `/joy`, `/speech_recognition`, `/tts`, `/gripper/command` |
| System | `/diagnostics`, `/robot_state`, `/rosout` |
| Task | `/task_status`, `/behavior_tree/status` |
| Debug | `/debug_image`, `/costmap_debug`, `/path_debug`, `/tf_debug` |

**Stand-ins** (no custom pkg in automsgs): `/waypoint`→`PoseStamped`, `/people`→`Detection3DArray`,
`/semantic_map`→`OccupancyGrid`, `/gripper/command`→`Float64`, `/robot_state`/`/behavior_tree/status`→`String`,
`/rosout`→`foxglove.Log` JSON, `/move_base/status`→`action_msgs/GoalStatusArray`.

Fixed Frame=`map`. Enable Displays / panels as needed (Image, LaserScan, Map, Path, RobotModel, Markers, Table, Log, Diagnostics via Table).

## 19 — Geometry extras

| Display | Channel | Type |
|---|---|---|
| GridCells | `/fake/grid_cells` | `map_msgs/GridCells` |
| PointStamped | `/fake/point` | `geometry_msgs/PointStamped` |
| Polygon | `/fake/polygon` | `geometry_msgs/PolygonStamped` |
| TwistStamped | `/fake/twist` | `geometry_msgs/TwistStamped` |
| AccelStamped | `/fake/accel` | `geometry_msgs/AccelStamped` |

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/19_tutorial_geometry_extras.py
./build/autonomy/bin/autoviz
```

Fixed Frame=`map` (Twist/Accel use `base_link` — run §05 TF if needed).

## 20 — Scalar env + Gauge

| Channel | Type | Also used by |
|---|---|---|
| `/fake/illuminance` | `Illuminance` | Illuminance Display |
| `/fake/fluid_pressure` | `FluidPressure` | FluidPressure Display |
| `/fake/relative_humidity` | `RelativeHumidity` | RelativeHumidity Display |
| `/fake/gauge` | `Float64` | Gauge / Indicator · field `data` |

```bash
/usr/bin/python3 examples/python/20_tutorial_scalar_env.py
./build/autonomy/bin/autoviz
```

## 21 — Audio

Publishes `foxglove.RawAudio` JSON (`pcm-s16`, 16 kHz mono) on `/fake/audio`.

```bash
/usr/bin/python3 examples/python/21_tutorial_audio.py
./build/autonomy/bin/autoviz
```

Open Audio panel → select `/fake/audio`.

## 22 — Service Call

Hosts `/fake/echo` (`std_msgs/String` → `echo:<request>`).

```bash
/usr/bin/python3 examples/python/22_tutorial_service.py
./build/autonomy/bin/autoviz
```

Service Call panel → service `/fake/echo`, JSON request `{"data":"hello"}`.

## 23 — Map GPS

Publishes `/gps/fix` (`NavSatFix`) on a circular track (Shanghai).

```bash
/usr/bin/python3 examples/python/23_tutorial_map_gps.py
./build/autonomy/bin/autoviz
```

Map panel → add topic layer `/gps/fix`, Follow=`/gps/fix`.

## 24 — Teleop echo

Subscribes `/cmd_vel` and prints Twist (verify Teleop panel publish).

```bash
# Terminal A
/usr/bin/python3 examples/python/24_tutorial_teleop_echo.py
# Terminal B
./build/autonomy/bin/autoviz   # open Teleop, drive
```
