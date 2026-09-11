# 传感器手册

主配置：`config/autodriver_hardware.yaml`。厂商细项位于 `config/camera|<lidar>/<vendor>/`，经 `params_file` 合并（条目内字段覆盖文件内容）。术语参见 [术语](../guide/glossary.md)。

> 「已实现」表示 Registry 可创建采集驱动（仍可能依赖本机 SDK）。「占位」表示 stub 或仅 Attach，无真实采集数据路径。

| 模态 | YAML | backend | 状态 | 页 |
|---|---|---|---|---|
| 相机/点云/板载 IMU | `camera` 折叠 | `realsense` `orbbec` | 已实现 | [RealSense](camera/realsense.md) [Orbbec](camera/orbbec.md) |
| 2D 激光 | `lidar_2d` | `rplidar` | 已实现 | [RPLidar](lidar/rplidar.md) |
| 3D 激光 | `lidar_3d` | `velodyne` `hesai` `livox` | 已实现 | [Velodyne](lidar/velodyne.md) [Hesai](lidar/hesai.md) [Livox](lidar/livox.md) |
| IMU/GPS | `imu` `gps` | `serial` `can`（含板载） | 已实现 | [IMU/GPS](imu_gps.md) |
| Radar/Mic/双目/Range | 对应键 | stub / 仅 Attach | 占位 | [Stub](stubs.md) |

构建与运行：[快速开始](../guide/quickstart.md) · [使用](../guide/usage.md) · [配置](../guide/configuration.md)
