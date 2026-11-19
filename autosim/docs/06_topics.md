# 06 · 话题（Autolink 通道）

逻辑键由 `Config.channel_map()` 映射到 YAML 中的通道字符串；Runner 按使能开关创建 writer。

| 相关 | 链接 |
|------|------|
| 配置 | [05](05_configuration.md) |
| Autoviz | [08](08_map_and_viz.md) |

---

## 1. 默认通道表

| 方向 | 逻辑键 | 默认通道 | 类型（概念） |
|------|--------|----------|--------------|
| 入 | `cmd_vel` | `/cmd_vel` | TwistStamped |
| 出 | `odom` | `/odom` | Odometry |
| 出 | `scan` | `/scan` | LaserScan |
| 出 | `points` | `/points` | PointCloud2（3D lidar） |
| 出 | `rgb` | `/camera/rgb/image_raw` | Image（rgb8） |
| 出 | `depth` | `/camera/depth/image_raw` | Image（32FC1） |
| 出 | `camera_info` | `/camera/camera_info` | CameraInfo |
| 出 | `depth_points` | `/camera/depth/points` | PointCloud2 |
| 出 | `topdown_rgb` | `/camera/topdown/image_raw` | Image（俯视 rgb8，可选） |
| 出 | `topdown_info` | `/camera/topdown/camera_info` | CameraInfo（可选） |
| 出 | `chase_rgb` | `/camera/chase/image_raw` | Image（第三人称跟随 rgb8，可选） |
| 出 | `chase_info` | `/camera/chase/camera_info` | CameraInfo（可选） |
| 出 | `imu` | `/imu` | Imu |
| 出 | `stereo_left_rgb` / `stereo_right_rgb` | `/camera/left\|right/image_raw` | Image（rgb8，可选） |
| 出 | `stereo_left_info` / `stereo_right_info` | `/camera/left\|right/camera_info` | CameraInfo（可选） |
| 出 | `range_0` … `range_n` | `/tof/*`、`/cliff/*` | Range（顺序同 `sensors.range.sensors`） |
| 出 | `joint_states` | `/joint_states` | JointState（可选） |
| 入 | `joint_command` | `/joint_commands` | JointState（可选） |
| 出 | `tf` / `tf_static` | `/tf` `/tf_static` | TFMessage |
| 出 | `clock` | `/clock` | Time |
| 出 | `footprint` | `/footprint` | PolygonStamped |
| 出 | `map_grid` | `/map` | OccupancyGrid |
| 出 | `map_cloud` | `/overall/map` | PointCloud2（采样一次，缓存重发） |
| 出 | `robot_description` | `/robot_description` | String（URDF XML，有 urdf 时） |
| 出 | `gt_pose` | `/gt/pose` | PoseStamped（默认关） |
| 出 | `gt_odom` | `/gt/odometry` | Odometry（可选，与 `gt_pose` 同时间戳，`map -> base_link`） |

具体以当前 YAML 为准；改通道名后需保证全局唯一（`validate` 会查重）。

---

## 2. 时间与坐标系

| 约定 | 说明 |
|------|------|
| RGB / depth / CameraInfo | 同一 `header.stamp`（Runner 硬同步） |
| `gt_pose` / `gt_odom` | 同一仿真周期和时间戳，平面 x/y/yaw 一致；Odometry 为 `map -> base_link`，并包含 `base_link_height` |
| `/tf_static` | 时间戳固定为 `(0, 0)`（tf2 无时效静态边）；Atlas 对非零静态戳按 ≤100 ms 校验，Habitat 卡顿时否则会丢弃 `map→odom` |
| Fixed Frame（Autoviz） | `nav` 下用 `map` |
| 相机光学 | 深度点云可经 `optical_to_camera_link` 转到 `camera_link` |

---

## 3. 使能关系（简）

- `sensors.camera.enabled: false` 时不建 rgb/depth writer；可仅保留 `depth_points_channel`（Fake / Isaac 特例）。
- `sensors.topdown.enabled: true` 时建 `topdown_rgb` / `topdown_info`（机载俯视 RGB）。
- `sensors.chase.enabled: true` 时建 `chase_rgb` / `chase_info`（第三人称跟随；自动 instance URDF，仅 chase 帧可见）。
- `map.enabled && map.publish` 时建 `map_cloud`（`/overall/map`）；`map_grid`（`/map`）仅在 `mode: nav` 下创建（`slam` 留给 Atlas / Cartographer）。
- `lidar_3d.enabled` 才发 `/points`。
- `robot.truth.enabled` 启用 `/gt/pose`；仅当 `robot.truth.odometry_channel` 非空时同时创建 `gt_odom` writer。

---

## 4. 关节与机器人专用通道

`sensors.stereo.enabled` 建左右目图像和 `camera_info`。右目 `P[3] = -fx × baseline_m`，符合校正立体惯例；左右图共用同一 `header.stamp`。

`sensors.range.enabled` 按 `sensors.range.sensors` 的书写顺序建 `range_0` … `range_n`，每个键对应一条 `sensor_msgs/Range`。Habitat-Sim 没有测距传感器，后端沿该坐标系 URDF `+x` 轴投射单条射线并跳过机器人自身碰撞体；无回波按 ROS 惯例报 `inf`。所有光束共用同一时间戳，按最慢的 `rate_hz` 节流。

`robot.joint_command.enabled` 建 `joint_command` 读端，`sensors.joint_states.enabled` 建 `joint_states` 写端。活动关节走 `/tf`，固定关节走 `/tf_static`，`/robot_description` 发布带相对 mesh 路径的 URDF XML。
