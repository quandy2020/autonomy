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
| `lidar/` | Lidar 基类、queue、scan_cut、compensator、`velodyne/` `hesai/` `livox/` `rplidar/`、stubs |

## 总览

| Module | backends | 消息 | 状态 |
|---|---|---|---|
| `ImuModule` | `serial`、`can`、`realsense` | `Imu` | 真采集（serial 经 `Stream`） |
| `GpsModule` | `serial`、`can` | `NavSatFix` | 真采集（serial 经 `Stream`） |
| `CameraModule` | `realsense`、`orbbec`、`smartereye` | `Image` + camera_info | RealSense/Orbbec 真；smartereye stub |
| `PointCloudModule` | `realsense`、`orbbec` | `PointCloud2` | RealSense / Orbbec（需 SDK） |
| `Lidar3dModule` | `velodyne` / `udp`、`hesai` / `pandar`、`livox`；stub: rslidar/… | PointCloud2 | Velodyne + Hesai + Livox |
| `RadarModule` | `conti` | PointCloud2 占位 | **stub** |
| `MicrophoneModule` | `respeaker` | Image PCM 占位 | **stub** |
| `Lidar2dModule` | `rplidar` / `slamtec` | LaserScan | Slamtec RPLidar A1/A2/A3 |
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

## Velodyne / Hesai / Livox / RPLidar

- **Velodyne**（`velodyne`/`udp`）：UDP→队列→切帧→Convert；校准 rad。详见 [传感器·Velodyne](../sensor/lidar/velodyne.md)。  
- **Hesai**（`hesai`/`pandar`）：XT32；校准 deg。详见 [Hesai](../sensor/lidar/hesai.md)。  
- **Livox**（`livox`）：SDK1/SDK2；详见 [Livox](../sensor/lidar/livox.md)。  
- **RPLidar**（`rplidar`/`slamtec`）：2D 串口 SDK；详见 [RPLidar](../sensor/lidar/rplidar.md)。  

点云约定：`point_step=24`。RAW_PACKET：`PushRawPacket`/`PushScan`。补偿：`enable_compensator` + `compensator.pose_channel`。

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

## realsense

需 `AUTODRIVER_WITH_REALSENSE=ON` + librealsense2。同机多流经 `device_hub`。源码：`camera/realsense/`。用 `model`/`serial`/`index` 选机，**勿**为型号新建 backend。详见 [RealSense](../sensor/camera/realsense.md)。

## orbbec

需 `AUTODRIVER_WITH_ORBBEC=ON` 且找到 OrbbecSDK。同机多流经 `camera/orbbec/device_hub` 合并。源码：`camera/orbbec/`。

| `params` | 说明 |
|---|---|
| `stream` | `color` / `depth` / `left_ir` / `right_ir`（`ir`→left；`ir0` 单 IR） |
| `serial` / `index` / `model` | 选设备 |
| `width` / `height` / `fps` | `0` = SDK 默认档（对齐 ROS2 `:=0` / `OB_*_ANY`） |
| `frame_id` | 光学系覆盖 |
| `enable_laser` | IR 投影灯；Gemini 330 官方默认 **true** |
| `device_preset` | 官方默认 **`Default`**（勿默认改成 High Accuracy） |
| `disparity_to_depth_mode` | **`HW`** / `SW` / `disable`（官方 HW） |
| `enable_disparity_to_depth` | 后处理 `DisparityTransform`；官方 **true** |
| `enable_hardware_noise_removal_filter` | 官方 **false** |
| `enable_noise_removal_filter` | 软去噪；官方 **true** |
| `noise_removal_filter_min_diff` / `max_size` | 官方 **256** / **80** |
| `enable_spatial_filter` | 官方 **false** |

**Channel 对齐 OrbbecSDK_ROS2 Gemini 330**（`camera_name:=camera`）：  
`/camera/color|depth|left_ir|right_ir/image_raw`、`…/camera_info`、点云
`/camera/depth/points`（默认）或 `/camera/depth_registered/points`。  
示例：`config/camera/orbbec/gemini_330.yaml`（仅设备 params；由
`autodriver_hardware.yaml` 的 `params_file` 合并）。
加载：`autodriver $AUTODRIVER_PATH autodriver_hardware.yaml`。

无 SDK 时 `Create` 返回 `nullptr` 并打日志。

## 扩展真 Camera

1. `camera/<vendor>/` 实现 hub + drivers  
2. `REGISTER_CAMERA_BACKEND` / `REGISTER_POINTCLOUD_BACKEND`  
3. CMake 加入源（可选 `AUTODRIVER_WITH_*`）  
4. **不必**改 `CameraModule` / `PointCloudModule`

## 扩展真 Lidar

2D：`lidar/rplidar/` + `REGISTER_LIDAR2D_BACKEND`。  
3D：`REGISTER_LIDAR_BACKEND`；参考 `velodyne/`、`hesai/`、`livox/`。点云 `point_step=24`。不必改 `Lidar*Module`。

## 外置插件（高级）

`library` 非空时从 `plugin_dir` 加载 `.so`。常规部署用内置 modules 即可。
