# Hesai lidar (PandarXT / XT32)

Backend: `hesai`（alias `pandar`）。

Pipeline 与 Velodyne 相同：UDP → `PacketQueue` → 方位角切帧 → Convert →
optional compensator → `PointCloud2`（`point_step=24`）。

| 文件 | 职责 |
|---|---|
| `packet.hpp` | 1080B XT32 UDP 布局（manual §3.1） |
| `convert.*` | 仰角表（默认 +15°…−16°；可被 calibration YAML 覆盖） |
| `calibration.*` | `vert_correction` YAML（度） |
| `udp_driver.*` | online / `raw_packet` / `PushScan` |

默认 `model: XT32`。未知型号告警后仍用 XT32 角度。XT32M2X（6-block / 5 mm）未覆盖。

常用 `params`：`data_port`、`packets_per_scan`、`use_azimuth_cut`、
`scan_cut_angle_deg`、`packet_queue_capacity`、`enable_compensator`、
`calibration_path`（见 `config/params/XT32_calibration.yaml`，仰角为度）、
`publish_scan`。

回放：`PushRawPacket` / `PushScan(LidarPacketScan)`（整帧只出点云）。
