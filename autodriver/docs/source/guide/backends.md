# 后端

传感器插件通过 `backend` 选择硬件驱动。模块在 `modules.cpp` 注册并编入 `libautodriver`；传输与协议尽量分离（`Stream` × `Parser` 分离）。

## 目录与职责

| 目录 | 内容 |
|---|---|
| `common/` | `Stream`、`SerialPort`、`CanSocket`、`LoadExtrinsicYaml`、`DeviceStatus` |
| `canbus/` | ProtocolData、Receiver、Client（Socket+Fake）、Sender、`byte` |
| `imu/` | WitMotion parser、serial/CAN IMU driver |
| `gps/` | NMEA parser、`gps/parser` 工厂、serial/CAN GPS |
| `camera/` | `backend_registry`、`realsense/`、`orbbec/` |
| `smartereye/` | Camera backend stub |
| `radar/` | Registry + Conti stub |
| `microphone/` | Registry + Respeaker stub |
| `lidar/` | `SourceType`、`LidarComponentBase`、`packet_queue`、`scan_cut`、`MotionCompensator`、`velodyne/`、`hesai/`、厂商 stub |

## 总览

| Module | backends | 消息 | 状态 |
|---|---|---|---|
| `ImuModule` | `serial`、`can`、`realsense` | `Imu` | 真采集（serial 经 `Stream`） |
| `GpsModule` | `serial`、`can` | `NavSatFix` | 真采集（serial 经 `Stream`） |
| `CameraModule` | `realsense`、`orbbec`、`smartereye` | `Image` + camera_info | RealSense/Orbbec 真；smartereye stub |
| `PointCloudModule` | `realsense`、`orbbec` | `PointCloud2` | RealSense / Orbbec（需 SDK） |
| `Lidar3dModule` | `velodyne` / `udp`、`hesai` / `pandar`；stub: livox/rslidar/… | PointCloud2 | Velodyne + Hesai XT32 |
| `RadarModule` | `conti` | PointCloud2 占位 | **stub** |
| `MicrophoneModule` | `respeaker` | Image PCM 占位 | **stub** |
| `Lidar2dModule` | — | LaserScan | **attach-only** |
| `RangeModule` | — | `Range` | **attach-only** |

## Stream（传输）

```cpp
#include "autodriver/common/stream.hpp"

auto stream = autodriver::common::CreateSerialStream("/dev/ttyUSB0", 115200);
stream->Connect();
stream->Read(buf, n, timeout_ms);
```

- `Status`：`diagnostics::DeviceStatus`（`kOk` / `kDisconnected` / `kError`）
- 实现：`SerialStream`、`UdpStream`（`CreateUdpStream`）
- `ReconnectStream`：串口 / Velodyne UDP 读循环断线退避重连
- 后续可插：TCP / NTRIP；Parser（NMEA、WitMotion、Velodyne Convert）保持独立

## Velodyne（lidar_3d）

UDP packet → 聚合 Scan → `ConvertPacketsToPointCloud` → `PointCloud2`（字段 `x,y,z,intensity,timestamp`，`point_step=24`）。

- 源码：`lidar/velodyne/{packet,convert,udp_driver}`
- `source_type: raw_packet` 时用 `PushRawPacket` 回放（单测已覆盖）
- `enable_compensator: true` 时内置 `PoseBuffer`；`autodriver` 进程经 `bridge::PoseFeeder` 订阅 `compensator.pose_channel`（`nav_msgs/Odometry`）并 `PushLidarPose`
- `publish_scan: true` 时先发 `LidarPacketScan` 再发点云；`Publisher` 只转发 `LidarCloud`（Scan 仅走 SampleSink / 旁路回调）
- `calibration_path`：Velodyne `VLP16_calibration.yaml`（rad）；Hesai `XT32_calibration.yaml`（deg）
- 静态外参：`common::LoadExtrinsicYaml`；示例 `config/params/lidar_vlp16_extrinsics.yaml`
- Hesai：`backend: hesai`（alias `pandar`）；**PandarXT / XT32** Convert 已实现（1080 字节包，4 mm）；XT32M2X 未覆盖
- Online：`PacketQueue` 收转解耦；默认 `use_azimuth_cut` + `packets_per_scan` 上限
- RAW_PACKET：`PushRawPacket`（按包）或 `PushScan`/`LidarComponentBase::InjectScan`（整帧 Scan→Convert，不再发 Scan）
- Stream：`CreateTcpStream` / `CreateNtripStream`（差分 GNSS 输入）

## serial

| `params` / YAML | 说明 |
|---|---|
| `device`（YAML `port`） | 串口路径 |
| `baud`（YAML `baudrate`） | 波特率 |
| `accel_scale` / `gyro_scale` | IMU 换算（可选） |

## canbus

共享 CAN 层：

- `canbus::CanClient`：`SocketCanClient` / `FakeCanClient`（channel 以 `fake` 开头）
- `CanReceiver` + `ProtocolData` / `MessageManager`
- `CanSender`：周期发帧；`byte.hpp`：位域打包

IMU/GPS CAN 驱动参数：

| 字段 | 说明 |
|---|---|
| `interface` | 如 `can0`（或 `fake0` 单测） |
| `accel_can_id` / `gyro_can_id` | IMU 分帧 ID |
| `accel_scale` / `gyro_scale` | IMU 换算（可选） |
| `can_id` | GPS lat/lon 帧 ID |

## radar / microphone / smartereye

| YAML | Module | backend | 状态 |
|---|---|---|---|
| `radar` | `RadarModule` | `conti`（alias `continental`） | stub：`Create`→nullptr |
| `microphone` | `MicrophoneModule` | `respeaker` | stub：需 PortAudio |
| `camera` + `backend: smartereye` | `CameraModule` | `smartereye` | stub：需厂商 SDK |

样本占位：`RadarSample`=`PointCloud2`，`MicrophoneSample`=`Image`（PCM 字节袋）。扩展步骤同 lidar：实现 Create + ProtocolData/SDK，不必改 Module 类。

## GNSS Parser 工厂

```cpp
#include "autodriver/gps/parser/parser.hpp"

auto parser = autodriver::gps::GnssParserRegistry::Instance().Create("nmea");
```

- 内置：`nmea` / `nmea0183` → `Nmea0183Parser`（行缓冲 GGA/RMC）
- 与 `common::Stream` 搭配：读循环 `Consume(buf, n)` → 可选 `ParsedFix`
- 串口 GPS 驱动仍可直接用 `nmea_0183.hpp`；工厂便于后续厂商二进制协议

## Lidar PacketQueue

`lidar/packet_queue.hpp`：有界 FIFO，满则丢最旧并累计 `dropped()`。  
**Velodyne / Hesai online 路径已接入**（ReadLoop → queue → ProcessLoop）。`PushRawPacket` 不经队列。

切帧见 `lidar/scan_cut.hpp`：`use_azimuth_cut`（默认 true）+ `packets_per_scan` 上限。

需 `AUTODRIVER_WITH_REALSENSE=ON` 且找到 librealsense2。同机多流经 `camera/realsense/device_hub` 合并。源码：`camera/realsense/`。多型号用 `params.model` / `serial` / `index` 区分，**不要**为型号新建 backend。

## orbbec

需 `AUTODRIVER_WITH_ORBBEC=ON` 且找到 OrbbecSDK。同机多流经 `camera/orbbec/device_hub` 合并。源码：`camera/orbbec/`。

| `params` | 说明 |
|---|---|
| `stream` | `color` / `depth` / `ir` |
| `serial` / `index` / `model` | 选设备 |
| `width` / `height` / `fps` | 分辨率与帧率 |
| `frame_id` | 光学系覆盖 |

无 SDK 时 `Create` 返回 `nullptr` 并打日志。

## 扩展真 Camera

1. `camera/<vendor>/` 实现 hub + drivers  
2. `REGISTER_CAMERA_BACKEND` / `REGISTER_POINTCLOUD_BACKEND`  
3. CMake 加入源（可选 `AUTODRIVER_WITH_*`）  
4. **不必**改 `CameraModule` / `PointCloudModule`

## 扩展真 Lidar

已落地参考实现：`lidar/velodyne/`（`backend: velodyne`，别名 `udp`）。

新增厂商：

1. 在 `lidar/<vendor>/` 实现 packet / convert / UDP（或 SDK）driver  
2. `REGISTER_LIDAR_BACKEND(tag, "name", CreateFn, "alias"...)`（见 `lidar/backend_register.hpp`）  
3. CMake 把源文件编入 `libautodriver`  
4. **不必**改 `Lidar3dModule`（查 `LidarBackendRegistry`）

点云字段约定：`x,y,z,intensity,timestamp`（`point_step=24`），以便复用 `MotionCompensator`。

## 外置插件（高级）

`library` 非空时从 `plugin_dir` 加载 `.so`。常规部署用内置 modules 即可。
