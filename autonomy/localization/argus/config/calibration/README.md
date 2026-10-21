# Calibration configs

Camera intrinsics and sensor extrinsics live here (not in the main LIVO YAML).

| File | Content |
|------|---------|
| `camera_pinhole.yaml` | Synth / TurtleBot example intrinsics |
| `camera_spires_equidistant.yaml` | Oxford Spires cam0 (DiffCal equidistant) |
| `extrinsic_lidar_camera.yaml` | Synth / TurtleBot LiDAR → camera |
| `extrinsic_lidar_camera_spires.yaml` | Spires DiffCal cam0 `T_cam_lidar` |
| `extrinsic_imu_lidar.yaml` | Synth / TurtleBot LiDAR → IMU |
| `extrinsic_imu_lidar_spires.yaml` | Spires DiffCal `T_IL = T_CI^{-1} T_CL` |

Oxford Spires (`livo_esikf_spires.yaml` / `livo_esikf_accurate.yaml`):

```yaml
camera:
  calib_file: calibration/camera_spires_equidistant.yaml
extrinsic:
  lidar_camera: calibration/extrinsic_lidar_camera_spires.yaml
  imu_lidar: calibration/extrinsic_imu_lidar_spires.yaml
```

TurtleBot / autosim (`livo_esikf.yaml` / `livo_autosim.yaml`):

```yaml
camera:
  calib_file: calibration/camera_pinhole.yaml
extrinsic:
  lidar_camera: calibration/extrinsic_lidar_camera.yaml
  imu_lidar: calibration/extrinsic_imu_lidar.yaml
```

Paths are relative to the main YAML directory (`config/`).
