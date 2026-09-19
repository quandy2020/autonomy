# LIVO Naming Refactor (cartographer mid-length)

**Goal:** Concise `snake_case` files / `PascalCase` types; functions as VerbNoun PascalCase. Keep `bridge/`. No algorithm changes.

See plan todos in session; mapping tables below are the source of truth for this pass.

## Files / types

| Old path / type | New |
|-----------------|-----|
| `bridge/node_runner` | `bridge/livo_runner` (`LivoRunnerFlags`, `RunLivo`) |
| `bridge/*_message_converter` | `bridge/msg_conversion` free functions |
| `sensor/lidar_point_cloud_preprocessor` | `sensor/point_cloud_preprocessor` / `PointCloudPreprocessor` |
| `sensor/inertial_measurement_preprocessor` | `sensor/imu_processor` / `ImuProcessor` |
| `mapping/mapping_orchestrator` | `mapping/map_builder` / `MapBuilder` / `MapBuilderOptions` |
| `mapping/visual/visual_odometry_manager` | `mapping/visual/vio_manager` / `VisualOdometry` |
| `mapping/visual/visual_point_voxel_bucket` | `mapping/visual/voxel_points` / `VisualVoxelBucket` |
| `LidarVoxelMap` | `LidarVoxelMap` |
| `common/camera_model_loader` | `common/load_camera` / `LoadPinholeCamera` |
| `common/voxel_map_configuration_loader` | `common/load_voxel_map` / `LoadVoxelMapConfig` |
| `types/estimator_state` → `States` | `types/states` |
| `types/visual_inertial_measurement_group` | `types/measure_group` / `MeasureGroup` |
| `types/synchronized_sensor_package` | `types/lidar_measure_group` / `LidarMeasureGroup` |
| `types/point_with_covariance` | `types/point_with_var` / `PointWithVar` |
| `types/slam_operating_mode` | `types/slam_mode` / `SlamMode` |
| `types/estimation_pipeline_stage` | `types/ekf_stage` / `EkfStage` |
| `types/lidar_sensor_model` | `types/lidar_type` / `LidarType` |
| `types/message_stamp` | `types/time` |

## Functions (selected)

`MapBuilder`: `SyncPackages`, `EstimateAndMap`, `HandleVio`/`HandleLio`, `AlignGravity`, `ProcessImu`, `PropagateImuOnce`/`PropagateImuLoop`, `TransformLidar`, `TransformPointBodyToWorld`, …

`SensorBridge`: `HandlePointCloud2Message`, `HandleImuMessage`, `HandleImageMessage`

`ImuProcessor`: `SetExtrinsic`, `Process`, `UndistortPointCloud`, `InitializeImu`, …
