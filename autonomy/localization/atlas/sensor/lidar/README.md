# Lidar sensor (Atlas)

ROS / Autolink ingress and cloud pipeline for lidar.

| File / dir | Role |
|------------|------|
| `lidar_bridge.*` | PointCloud2 → preprocess → deskew → `FeedWithPose` / loop |
| `lidar_sensor.*` | Cloud queue + residual feed into Atlas estimator |
| `preprocess.*` | range / blind / voxel · `LidarModel` · TimedPoint |
| `obs_model.*` | `BuildAgainstIVox` → point-plane residual batch |

Moved out:

| New path | Role |
|----------|------|
| `frontend/eskf/` | IEKF + analytic H for LocalEstimator |
| `frontend/lio/` | MeasureGroup · ImuProcess trajectory deskew · Sync |
| `mapping/ivox/` | voxel hash + PCA plane (owned via MapIncremental) |
| `mapping/tiled_map.hpp` | chunked tile map |
| `backend/lidar_loop_detector.*` | PCL multi-res NDT + point-plane ICP fallback |
| `backend/lidar_pose_graph.*` | LO/LIO SE3 pose graph (g2o); not vision LoopClosing |

Config: `conf/atlas/lightning/default.yaml` (lidar preprocess / obs defaults).
