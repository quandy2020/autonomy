# Hesai lidar

Backend: `hesai`（alias `pandar`）。

两条路径由工厂 `CreateHesaiLidarDriver` 分发：

| 路径 | 何时选用 | 依赖 |
|---|---|---|
| **UDP（默认）** | `model` 为 XT32 / PandarXT 族，且未设 `use_sdk` | 无第三方 SDK |
| **SDK** | `use_sdk: true` / `parser: sdk` / 非 XT32 型号 | [HesaiLidar_SDK_2.0](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0) |

点云输出统一为 `PointCloud2`（`point_step=24`，x/y/z/intensity/timestamp）。

## UDP（PandarXT / XT32）

Pipeline：UDP → `PacketQueue` → 方位角切帧 → Convert → optional compensator。

| 文件 | 职责 |
|---|---|
| `packet.hpp` | 1080B XT32 UDP 布局（manual §3.1） |
| `convert.*` | 仰角表（默认 +15°…−16°；可被 calibration YAML 覆盖） |
| `calibration.*` | `vert_correction` YAML（度） |
| `udp_driver.*` | online / `raw_packet` / `PushScan` |

常用 `params`：`data_port`（亦接受 ROS 名 `udp_port`）、`packets_per_scan`、
`use_azimuth_cut`、`scan_cut_angle_deg`、`calibration_path`、`publish_scan`。
设备示例：`config/lidar/hesai/xt32.yaml`。

## SDK（兼容 HesaiLidar_ROS_2.0）

与官方 ROS2 驱动相同的底层 SDK（`HesaiLidarSdk<LidarPointXYZIRT>`），参数名对齐
`HesaiLidar_ROS_2.0/config/config.yaml` 的扁平键（autodriver 的 `params` /
`params_file` 为 flat map，无需嵌套 `lidar_udp_type`）。

| 文件 | 职责 |
|---|---|
| `sdk_driver.*` | Init/Start/Stop、帧回调 → XYZIT PointCloud2 |

启用：

```bash
./scripts/install_hesai_lidar_sdk.sh /path/to/HesaiLidar_SDK_2.0
# CMake
-DAUTODRIVER_WITH_HESAI_SDK=ON
```

关键参数：`device_ip_address`、`udp_port`、`ptc_port`、`correction_file_path`、
`firetimes_path`、`use_ptc_connected`、`hesai_source_type`（1=实车 UDP，2=pcap，…）。

设备示例：

| 配置 | 型号 | 说明 |
|---|---|---|
| `config/lidar/hesai/sdk.yaml` | 通用（如 OT128） | SDK 参数模板 |
| `config/lidar/hesai/jt128.yaml` | JT128 | 以太网 UDP；含 `socket_buffer_size` 等 |

JT128 走 SDK 路径（`model: JT128` 会自动 `WantHesaiSdk`；也可显式 `use_sdk: true`）。
串口 `rs485_com` / `rs232_com` 仅用于 JT16（`hesai_source_type: 4`），JT128 不要用。

说明：ROS2 点云含 `ring` 字段；autodriver 为与 Velodyne/Livox 一致，输出不含
`ring` 的 XYZIT。SDK 点时间戳（秒）会换算为纳秒写入 `timestamp`。
