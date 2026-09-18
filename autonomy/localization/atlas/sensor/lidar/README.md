# Lidar sensor (Atlas)

ROS / Autolink ingress and cloud pipeline for lidar.

| File / dir | Role |
|------------|------|
| `lidar_bridge.*` | PointCloud2 → preprocess → `FeedWithPose` |
| `lidar_sensor.*` | Cloud queue + residual feed into Atlas estimator |
| `preprocess.*` | range / blind / voxel (`sensor::lightning`) |
| `obs_model.*` | `BuildAgainstIVox` → residual batch (`sensor::lightning`) |

Moved out (path only; namespace still `sensor::lightning` temporarily):

| New path | Role |
|----------|------|
| `frontend/eskf/` | LO/LIO iterate strategy for LocalEstimator |
| `mapping/ivox/` | voxel hash + PCA plane (owned via MapIncremental) |

Config: `conf/atlas/lightning/default.yaml`.
