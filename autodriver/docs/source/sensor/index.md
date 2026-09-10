# 传感器手册

主配置：`config/autodriver_hardware.yaml`。厂商细项：`config/camera|<lidar>/<vendor>/`，经 `params_file` 合并（条目内覆盖文件）。

| 模态 | YAML | backend | 状态 | 页 |
|---|---|---|---|---|
| 相机/点云/板载 IMU | `camera` 折叠 | `realsense` `orbbec` | 真 | [RealSense](camera/realsense.md) [Orbbec](camera/orbbec.md) |
| 2D 激光 | `lidar_2d` | `rplidar` | 真 | [RPLidar](lidar/rplidar.md) |
| 3D 激光 | `lidar_3d` | `velodyne` `hesai` `livox` | 真 | [Velodyne](lidar/velodyne.md) [Hesai](lidar/hesai.md) [Livox](lidar/livox.md) |
| IMU/GPS | `imu` `gps` | `serial` `can` (+板载) | 真 | [IMU/GPS](imu_gps.md) |
| Radar/Mic/双目/Range | 对应键 | stub / attach-only | 占位 | [Stub](stubs.md) |

构建与运行：[快速开始](../guide/quickstart.md) · [使用](../guide/usage.md) · [配置](../guide/configuration.md)
