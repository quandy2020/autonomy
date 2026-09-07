# lidar

3D lidar backends for `Lidar3dModule`（按厂商分包的 lidar 后端布局）。

## 管线

```
UDP ReadLoop → PacketQueue → ProcessLoop → 方位角切帧 / packets_per_scan
    → optional LidarPacketScan → Convert → optional MotionCompensator → PointCloud2
```

`PushRawPacket`（单包）与 `PushScan` / `InjectScan`（整帧 `LidarPacketScan`）绕过队列；后者只 Convert→点云，不再次 `WriteScan`。

## 目录

| 路径 | 状态 |
|---|---|
| `backend_registry.*` / `backend_register.hpp` | `backend` → 工厂 |
| `scan_cut.hpp` | 方位角 wrap / cut 判定 |
| `packet_queue.hpp` | 有界丢旧 FIFO（online 收转解耦） |
| `lidar_component_base.hpp` | Scan / Convert 钩子 |
| `motion_compensator.*` / `pose_buffer.*` / `motion_pose_sink.hpp` | 扫面内运动补偿；`SensorManager::PushLidarPose` |
| `velodyne/` | **真采集**（VLP-16 + calibration YAML） |
| `hesai/` | **真采集**（PandarXT / XT32 + 可选 elevation YAML） |
| `vendor_stubs.cpp` | livox / rslidar / lslidar / seyond / vanjee → nullptr |
| `source_type.hpp` | `online` \| `raw_packet` |

## backends

| backend | alias | 说明 |
|---|---|---|
| `velodyne` | `udp` | UDP 2368，默认 VLP-16 |
| `hesai` | `pandar` | UDP 2368，默认 XT32 |
| `livox` | — | stub |
| `rslidar` | `robosense` | stub |
| `lslidar` | — | stub |
| `seyond` | — | stub |
| `vanjee` | `vanjeelidar` | stub |

**主从点云 fusion 不是 lidar_3d backend**；多传感器对齐走 `SensorHub` / 后续独立 fusion 模块。

## 切帧与队列参数

| `params` | 默认 | 说明 |
|---|---|---|
| `use_azimuth_cut` | `true` | 末 block 方位角跨 cut 则 Emit |
| `scan_cut_angle_deg` | `0` | cut 角度（度） |
| `packets_per_scan` | 75 / 180 | **上限保险**（无 wrap 时仍可按包数切） |
| `packet_queue_capacity` | `256` | online 队列长度，满则丢最旧 |

## 运动补偿位姿

`enable_compensator: true` 时驱动实现 `lidar::MotionPoseSink`。

进程配置：

```yaml
compensator:
  pose_channel: /localization/odom
```

`autodriver` 进程内 `bridge::PoseFeeder` 订阅 `nav_msgs/Odometry`，对所有
`enable_compensator` 的 3D lidar 调用 `PushLidarPose`。可选
`params.pose_channel` 覆盖通道；`params.extrinsic_path` 存在时乘外参。

代码侧也可直接：

```cpp
manager.PushLidarPose("lidar/vlp16", time_ns, world_T_lidar);
```
