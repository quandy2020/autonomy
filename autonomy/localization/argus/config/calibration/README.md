# Calibration configs

Camera intrinsics and sensor extrinsics live here (not in the main LIVO YAML).

| File | Content |
|------|---------|
| `camera_pinhole.yaml` | FAST-LIVO2-style camera intrinsics (example) |
| `extrinsic_lidar_camera.yaml` | LiDAR → camera (`Rcl` 3×3 row-major, `Pcl` 3) |
| `extrinsic_imu_lidar.yaml` | LiDAR → IMU (`ext_R` / `ext_T`) |

Referenced from `config/livo_esikf.yaml`:

```yaml
camera:
  calib_file: calibration/camera_pinhole.yaml
extrinsic:
  lidar_camera: calibration/extrinsic_lidar_camera.yaml
  imu_lidar: calibration/extrinsic_imu_lidar.yaml
```

Paths are relative to the main YAML directory (`config/`).
