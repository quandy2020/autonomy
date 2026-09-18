# Lightning algorithms (embedded)

| Dir | Role |
|-----|------|
| `preprocess/` | range / blind / voxel |
| `ivox/` | voxel hash + PCA plane |
| `obs_model/` | `BuildAgainstIVox` → residual batch |
| `eskf/` | LO/LIO iterate strategy for LocalEstimator |

`LidarBridge` (parent dir) subscribes PointCloud2 → preprocess → `FeedWithPose`.  
Config: `conf/atlas/lightning/default.yaml`.
