# util/calibration

多传感器标定与畸变参数的统一入口（**不是**第二个 SLAM）。

| 内容 | 用途 |
|------|------|
| `CameraIntrinsics` + `distortion` | 相机光学畸变；`undistort.hpp` 用 OpenCV；Tracking 仍走 `sensor/camera` |
| `ImuIntrinsics` | ESKF 噪声 / 重力 / bias 先验 |
| `LidarIntrinsics` | 型号 / 量程 / `scan_period`；**运动去畸变**见 `frontend/lio`（IMU 轨迹） |
| `ExtrinsicSE3` | `T_imu_lidar`、`T_cam_imu`、`T_cam_lidar`、`T_base_wheel` |
| `CalibrationBundle` | YAML 加载；`ApplyTo(LocalEstimator)`；`ToExtrinsics()` |

## YAML

见 `conf/atlas/calibration/default.yaml`。Runtime profile 可：

```yaml
calibration_path: conf/atlas/calibration/default.yaml
# 或内联
calibration:
  camera: { ... }
  extrinsics: { T_imu_lidar: { R: [...], t: [...] } }
```

## 约定

- `T_imu_lidar`：`p_imu = R * p_lidar + t`（deskew / ObsModel）
- `T_cam_imu`：`p_cam = R * p_imu + t`
- 未给 `T_cam_lidar` 时：`T_cam_imu * T_imu_lidar`
- 时间偏置：`sensor_time = host_time + offset`
