# Voxel-SLAM 管线（Argus）

无 ROS 的 Voxel-SLAM 双管线，实现集中在 `argus/system/voxel/`。

| 模块 | 路径 |
|------|------|
| 状态 / 点协方差 | `imu_state` · `point_var` · `point_cov` |
| 体素键 / 点簇（已提升） | `mapping/voxel_key` · `mapping/point_cluster` |
| 下采样 / 地图多线程 | `downsample`（含 `DownsampleClose`）· `map_ops` |
| IMU 预积分 / EKF / 初始化 | `imu_preintegration` · `imu_ekf` · `init` |
| KdTree 里程计 | `kdtree_odom` |
| 滑窗八叉树 / LI-BA / OctreeGBA | `sliding_octree` · `lidar_factor` · `li_ba_optimizer` · `octree_gba` · `hierarchical_ba` |
| 回环 / 多 session | `loop_update` · `pgo_edges` · `session_io` · `scan_pose` · `previous_map` · `keyframe_load` · `normal_icp` |
| 编排 | `odometry` · `odometry_config` · `load_config` |
| BTC | `matching/loop_closure/triangle_descriptor/` |

```bash
offline_voxel_slam --config config/voxel_slam.yaml --synth 50 --out_tum /tmp/vs.tum
```

## 对齐进度

已对齐（相对上游 Voxel-SLAM，不含 ROS/PCL 发布）：

- 点协方差、`MotionInit`、LIO-EKF、`MultiMarginalize`/`MultiRecut`、`OctreeGba`
- `KdTreeOdom`；triangle descriptor `SearchLoop` / `candidate_verify` / `PlaneGeometricVerification`
- **编排**：跨 session `SearchLoop` → `NormalIcp` → `PgoEdges`；同 session `ratio_drift` 门控
- **回环即时 ISAM2**：`relc_counts` / `curr_halt` / `prev_halt` / `jours` 门控 → `OptimizeIsam2(inject_gba=false)` → `dx` 写回窗 + `LoopUpdateMap`（对齐上游 `isOpt`）
- **HBA**：局部窗 `HbaAddEdge`→`gba_edges1` + submap；收尾 `gba_edges2` + `TopDownIsam2`（含 GBA 边）
- `ScanPose`、`previous_map_read`（`.bin`）、`keyframe_loading`、`DownsampleClose`
- session 位姿 / 二进制点云 I/O

PGO：默认 `ARGUS_USE_GTSAM=ON` 时走 **GTSAM ISAM2**（在线 `isOpt` + 收尾 `topDownProcess`）；无 GTSAM 时回退 Argus `PoseGraph`（Gauss–Newton）。无 ROS 可视化发布。
