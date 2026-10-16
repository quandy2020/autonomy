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
| 出 | `imu` | `/imu` | Imu |
| 出 | `tf` / `tf_static` | `/tf` `/tf_static` | TFMessage |
| 出 | `clock` | `/clock` | Time |
| 出 | `footprint` | `/footprint` | PolygonStamped |
| 出 | `map_grid` | `/map` | OccupancyGrid |
| 出 | `map_cloud` | `/overall/map` | PointCloud2（采样一次，缓存重发） |
| 出 | `robot_description` | `/robot_description` | String（URDF XML，有 urdf 时） |
| 出 | `gt_pose` | `/gt/pose` | PoseStamped（默认关） |

具体以当前 YAML 为准；改通道名后需保证全局唯一（`validate` 会查重）。

---

## 2. 时间与坐标系

| 约定 | 说明 |
|------|------|
| RGB / depth / CameraInfo | 同一 `header.stamp`（Runner 硬同步） |
| Fixed Frame（Autoviz） | `nav` 下用 `map` |
| 相机光学 | 深度点云可经 `optical_to_camera_link` 转到 `camera_link` |

---

## 3. 使能关系（简）

- `sensors.camera.enabled: false` 时不建 rgb/depth writer；可仅保留 `depth_points_channel`（Fake / Isaac 特例）。
- `map.enabled && map.publish`（且非 slam）才建 `map_grid` / `map_cloud`。
- `lidar_3d.enabled` 才发 `/points`。
