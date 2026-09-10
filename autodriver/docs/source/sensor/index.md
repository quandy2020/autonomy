# 传感器手册

按厂商与模态说明：简介、依赖、配置、运行要点。主配置仍是
[`config/autodriver_hardware.yaml`](../../../../config/autodriver_hardware.yaml)；
厂商细项放在 `config/camera/<vendor>/`、`config/lidar/<vendor>/`，经 `params_file` 合并。

## 能力一览

| 模态 | YAML 键 | 主要 backend | 状态 |
|---|---|---|---|
| 相机 / 点云 / 板载 IMU | `camera`（可折叠） | `realsense`、`orbbec` | 真采集 |
| 2D 激光 | `lidar_2d` | `rplidar` / `slamtec` | 真采集 |
| 3D 激光 | `lidar_3d` | `velodyne`、`hesai`、`livox` | 真采集 |
| IMU | `imu` | `serial`、`can`、`realsense` | 真采集 |
| GNSS | `gps` | `serial`、`can` | 真采集 |
| 毫米波 / 麦克 / 双目 | `radar` / `microphone` / smartereye | stub | 待 SDK |
| 超声波 | `range` | — | attach-only |

## 阅读顺序

1. [Intel RealSense](camera/realsense.md)
2. [Orbbec](camera/orbbec.md)
3. [Slamtec RPLidar](lidar/rplidar.md)
4. [Velodyne](lidar/velodyne.md)
5. [Hesai](lidar/hesai.md)
6. [Livox](lidar/livox.md)
7. [IMU / GPS](imu_gps.md)
8. [Stub 与占位](stubs.md)

通用构建与配置见 [快速开始](../guide/quickstart.md)、[配置](../guide/configuration.md)。
