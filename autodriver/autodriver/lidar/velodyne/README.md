# Velodyne lidar

Backend: `velodyne`（alias `udp`）。

Pipeline: UDP → `PacketQueue` → 方位角切帧 → optional `LidarPacketScan` →
`ConvertPacketsToPointCloud` → optional compensator → `PointCloud2`
（`x,y,z,intensity,timestamp`，`point_step=24`）。

| 文件 | 职责 |
|---|---|
| `packet.hpp` | 1206B firing 布局 |
| `calibration.*` | beam YAML / 默认 VLP-16 |
| `convert.*` | 包 → 点云 |
| `udp_driver.*` | online / `raw_packet` |

常用 `params`：`data_port`（2368）、`model`、`calibration_path`、
`use_azimuth_cut`、`scan_cut_angle_deg`、`packets_per_scan`、
`packet_queue_capacity`、`enable_compensator`、`publish_scan`、
`source_type`（`online` | `raw_packet`）。

回放：`PushRawPacket` 按包聚合；或 `PushScan(LidarPacketScan)` / `InjectScan` 整帧 Convert。

示例校准：`config/params/VLP16_calibration.yaml`。
