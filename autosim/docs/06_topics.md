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

<!-- BEGIN generated: config-channels -->

以 `config/default.yaml` 为基线，只列新增、改名或缺失的逻辑键；完整默认表见第 1 节。

### 顶层配置

| 配置 | 与 `default.yaml` 的差异 |
|------|--------------------------|
| `atlas.yaml` | `gt_odom=/gt/odometry`（新增）、`chase_rgb=/camera/chase/image_raw`（新增）、`chase_info=/camera/chase/camera_info`（新增）、`semantic`（缺）、`semantic_ids`（缺）、`depth_points`（缺） |
| `bev.yaml` | `surround_rgb_front=/surround/front/image_raw`（新增）、`surround_info_front=/surround/front/camera_info`（新增）、`surround_rgb_front_left=/surround/front_left/image_raw`（新增）、`surround_info_front_left=/surround/front_left/camera_info`（新增）、`surround_rgb_rear_left=/surround/rear_left/image_raw`（新增）、`surround_info_rear_left=/surround/rear_left/camera_info`（新增）、`surround_rgb_rear=/surround/rear/image_raw`（新增）、`surround_info_rear=/surround/rear/camera_info`（新增）、`surround_rgb_rear_right=/surround/rear_right/image_raw`（新增）、`surround_info_rear_right=/surround/rear_right/camera_info`（新增）、`surround_rgb_front_right=/surround/front_right/image_raw`（新增）、`surround_info_front_right=/surround/front_right/camera_info`（新增）、`surround_panorama=/surround/panorama`（新增）、`footprint`（缺）、`semantic`（缺）、`semantic_ids`（缺）、`depth_points`（缺）、`map_cloud`（缺） |
| `coverage.yaml` | `fov_marker=/camera/depth/fov`（新增）、`semantic`（缺）、`semantic_ids`（缺） |
| `coverage_rooms.yaml` | `fov_marker=/camera/depth/fov`（新增）、`semantic`（缺）、`semantic_ids`（缺） |
| `default.yaml` | 基线 |
| `fake.yaml` | `fov_marker=/camera/depth/fov`（新增）、`semantic`（缺）、`semantic_ids`（缺） |
| `fake_nav.yaml` | `fov_marker=/camera/depth/fov`（新增）、`semantic`（缺）、`semantic_ids`（缺） |
| `habitat_people.yaml` | `topdown_rgb=/camera/topdown/image_raw`（新增）、`topdown_info=/camera/topdown/camera_info`（新增） |
| `isaac.yaml` | `joint_states=/joint_states`（新增）、`hunav_agents=/hunav/agents`（新增）、`semantic`（缺）、`semantic_ids`（缺） |
| `isaac_nav.yaml` | `semantic`（缺）、`semantic_ids`（缺） |
| `lightning.yaml` | `odom_path=/odom/path`（新增）、`semantic`（缺）、`semantic_ids`（缺） |
| `lio.yaml` | `odom_path=/odom/path`（新增）、`semantic`（缺）、`semantic_ids`（缺） |
| `nav.yaml` | `chase_rgb=/camera/chase/image_raw`（新增）、`chase_info=/camera/chase/camera_info`（新增） |
| `slam.yaml` | 同 `default.yaml` |

### 机器人档案

机器人档案只列新增或改名的键；它们不是完整的导航栈，缺省键由 `_base.yaml` 或运行时补齐。

| 档案 | 与 `default.yaml` 的差异 |
|------|--------------------------|
| `google_robot.yaml` | `imu=/imu/data`、`stereo_left_rgb=/camera/left/image_raw`（新增）、`stereo_left_info=/camera/left/camera_info`（新增）、`stereo_right_rgb=/camera/right/image_raw`（新增）、`stereo_right_info=/camera/right/camera_info`（新增）、`range_0=/tof/right_1`（新增）、`range_1=/tof/right_2`（新增）、`range_2=/tof/right_back_1`（新增）、`range_3=/tof/right_back_2`（新增）、`range_4=/tof/left_1`（新增）、`range_5=/tof/left_2`（新增）、`range_6=/tof/left_back_1`（新增）、`range_7=/tof/left_back_2`（新增）、`range_8=/cliff/rear_right`（新增）、`range_9=/cliff/rear_left`（新增）、`joint_command=/joint_commands`（新增）、`joint_states=/joint_states`（新增）、`chase_rgb=/camera/chase/image_raw`（新增）、`chase_info=/camera/chase/camera_info`（新增） |
| `husky.yaml` | `imu=/imu/data`、`chase_rgb=/camera/chase/image_raw`（新增）、`chase_info=/camera/chase/camera_info`（新增） |
| `turtlebot3_burger.yaml` | `imu=/imu/data`、`chase_rgb=/camera/chase/image_raw`（新增）、`chase_info=/camera/chase/camera_info`（新增） |
| `unitree_g1.yaml` | `imu=/imu/data`、`joint_command=/joint_commands`（新增）、`joint_states=/joint_states`（新增）、`chase_rgb=/camera/chase/image_raw`（新增）、`chase_info=/camera/chase/camera_info`（新增） |
| `unitree_go2.yaml` | `imu=/imu/data`、`joint_command=/joint_commands`（新增）、`joint_states=/joint_states`（新增）、`chase_rgb=/camera/chase/image_raw`（新增）、`chase_info=/camera/chase/camera_info`（新增） |

<!-- END generated: config-channels -->

重新生成：`python3 scripts/gen_topics_doc.py`；`--check` 在测试里校验文档是否过期。

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
