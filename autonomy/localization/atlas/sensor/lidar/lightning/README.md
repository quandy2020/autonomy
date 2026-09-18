# Lightning algorithms (embedded)

Kept here (sensor-side cloud pipeline):

| Dir | Role |
|-----|------|
| `preprocess/` | range / blind / voxel · `TimedPoint` / `RunTimed` (P1) |
| `obs_model/` | `BuildAgainstIVox` → residual batch · ground PCA prior (P2) |

Moved out (path only; namespace still `sensor::lightning` temporarily):

| New path | Role |
|----------|------|
| `frontend/eskf/` | LO/LIO iterate + 15×15 `P_` (P0.3) |
| `frontend/lio/` | MeasureGroup · ImuProcess · LidarImuSync (P0.2) |
| `mapping/ivox/` | voxel hash + PCA plane + max_voxels LRU (P1) |
| `mapping/map_incremental.hpp` | `IntegrateScan` owns insert (P0.1) |
| `mapping/lidar_keyframe.hpp` | distance/angle keyframe gate (P1) |
| `backend/lidar_loop_detector.hpp` | NDT loop stub `Detect()→false` (P2) |

## P0 / P1 / P2 checklist

- [x] **P0.1** Residuals-only `FeedWithPose`; `MapIncremental::IntegrateScan` via `LidarBridge::SetMapIncremental`
- [x] **P0.2** Lidar–IMU sync + constant-ω deskew skeleton (pass-through if no point times / empty sync)
- [x] **P0.3** Eskf covariance propagate + coarse Joseph after GN (full lightning IEKF deferred)
- [x] **P1** `use_point_time` / `RunTimed`; IVox capacity eviction; lidar keyframes; `SetT_imu_lidar`
- [x] **P2** Loop stub + ground-plane PCA prior + docs

`LidarBridge` (parent dir): PointCloud2 → preprocess → optional deskew → `FeedWithPose` → `IntegrateScan` → `UpdateLidar`.  
Config: `conf/atlas/lightning/default.yaml`.
