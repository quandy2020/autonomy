# 后端

YAML `backend` → 对应 `*BackendRegistry` → Creator 建驱动。模态 `*Module` 固定编在 `libautodriver`（`modules.cpp`）；厂商只加驱动 + `REGISTER_*`。传输与协议尽量分离（`Stream` × `Parser`）。

| 相关 | 链接 |
|---|---|
| Registry / Creator 约定 | [架构](architecture.md) |
| YAML 字段 | [配置](configuration.md) |
| 厂商手册 | [传感器](../sensor/index.md) |
| 加宏示例 | [API](../api/overview.md) |

## 注册约定（速查）

```text
REGISTER_*_BACKEND(tag, "name", CreateFn, "alias"...)
  → NamedProductFactory → autolink::common::Factory
  → CreateDriver → SharedPtr
```

| 项 | 约定 |
|---|---|
| Creator | `Product*(const Id&, const DriverParams&)`，owning raw；stub 可 `nullptr` |
| 宏 | `REGISTER_IMU/GPS/CAMERA/POINTCLOUD/LIDAR/LIDAR2D/RADAR/MICROPHONE/CHASSIS_BACKEND` |
| 封装 | `common/named_factory.hpp` |

## 目录与职责

| 目录 | 内容 |
|---|---|
| `common/` | `Stream`、`SerialPort`、`CanSocket`、`calibration`、`named_factory` |
| `canbus/` | ProtocolData、Receiver、Client（Socket+Fake）、Sender、`byte` |
| `imu/` | WitMotion、serial/CAN；`backend_registry` |
| `gps/` | NMEA、`gps/parser`、serial/CAN；`backend_registry` |
| `camera/` | `backend_registry`、`realsense/`、`orbbec/` |
| `smartereye/` | Camera stub |
| `radar/` | Registry + Conti stub |
| `microphone/` | Registry + Respeaker stub |
| `lidar/` | 基类、queue、scan_cut、compensator、`velodyne/` `hesai/` `livox/` `rplidar/`、stubs |
| `bridge/` | `Publisher`、`PoseFeeder`、`channels.hpp` |
| `chassis/`（顶层） | `ChassisBackendRegistry`、`ChassisManager`、`stub/` |

## 总览

| Module / 编排 | Registry | backends | 消息 | 状态 |
|---|---|---|---|---|
| `ImuModule` | `ImuBackendRegistry` | `serial`、`can`、`realsense` | `Imu` | 真（serial→`Stream`） |
| `GpsModule` | `GpsBackendRegistry` | `serial`、`can` | `NavSatFix` | 真 |
| `CameraModule` | `CameraBackendRegistry` | `realsense`、`orbbec`、`smartereye` | `Image`（+ camera_info，若 `has_camera_info`） | RS/Orbbec 真；smartereye stub |
| `PointCloudModule` | `PointCloudBackendRegistry` | `realsense`、`orbbec` | `PointCloud2` | 需 SDK |
| `Lidar3dModule` | `LidarBackendRegistry` | `velodyne`/`udp`、`hesai`/`pandar`、`livox`；stub: rslidar/… | `PointCloud2` | 真 + stub |
| `Lidar2dModule` | `Lidar2dBackendRegistry` | `rplidar`/`slamtec` | `LaserScan` | RPLidar |
| `RadarModule` | `RadarBackendRegistry` | `conti`/`continental` | PointCloud2 占位 | **stub** |
| `MicrophoneModule` | `MicrophoneBackendRegistry` | `respeaker` | Image PCM 占位 | **stub** |
| `RangeModule` | — | — | `Range` | **attach-only** |
| `ChassisManager` | `ChassisBackendRegistry` | `stub` / 厂商 | Twist↔RobotState/Odom | stub 可联调 |

CMake：`AUTODRIVER_WITH_{REALSENSE,ORBBEC,RPLIDAR,LIVOX}`；未找到 SDK → 对应 Create→`nullptr`，不挡链接。

## Stream（传输）

```cpp
#include "autodriver/common/stream.hpp"

auto stream = autodriver::common::CreateSerialStream("/dev/ttyUSB0", 115200);
stream->Connect();
stream->Read(buf, n, timeout_ms);
```

| 项 | 说明 |
|---|---|
| 状态 | `diagnostics::DeviceStatus`：`kOk` / `kDisconnected` / `kError` |
| 实现 | `SerialStream`、`UdpStream`（`CreateUdpStream`） |
| 重连 | `ReconnectStream`：串口 / Velodyne UDP 读循环退避重连 |
| 预留 | TCP / NTRIP；Parser（NMEA、WitMotion、Convert）保持独立 |

## serial

| `params` / YAML | 说明 |
|---|---|
| `device`（YAML `port`） | 串口路径；建议 `/dev/serial/by-id/…` |
| `baud`（YAML `baudrate`） | 波特率 |
| `accel_scale` / `gyro_scale` | IMU 换算（可选） |

用于 IMU/GPS serial backend；RPLidar 走自有 SDK 通道参数（见下）。

## canbus

共享 CAN 层：

- `canbus::CanClient`：`SocketCanClient` / `FakeCanClient`（channel 以 `fake` 开头）
- `CanReceiver` + `ProtocolData` / `MessageManager`
- `CanSender`：周期发帧；`byte.hpp`：位域打包

IMU/GPS CAN 参数：

| 字段 | 说明 |
|---|---|
| `interface` | `can0` 或 `fake0`（单测） |
| `accel_can_id` / `gyro_can_id` | IMU 分帧 ID |
| `accel_scale` / `gyro_scale` | IMU 换算（可选） |
| `can_id` | GPS lat/lon 帧 ID |

上线前：`ip link set can0 up type can bitrate …`。

## GNSS Parser（与 GPS backend 正交）

传输后端由 `GpsBackendRegistry` 选择；**句解析**由 `GnssParserRegistry`（`NamedProductFactory0`）：

```cpp
#include "autodriver/gps/parser/parser.hpp"

auto parser = autodriver::gps::GnssParserRegistry::Instance().CreateParser("nmea");
// 读循环：parser->Consume(buf, n) → optional ParsedFix
```

| 名 | 实现 |
|---|---|
| `nmea` / `nmea0183` | `Nmea0183Parser`（行缓冲 GGA/RMC） |

扩展二进制协议：只加 Parser + `RegisterParser`，不必改 `GpsModule`。

## Velodyne / Hesai / Livox / RPLidar

| backend | 路径摘要 | 手册 |
|---|---|---|
| `velodyne` / `udp` | UDP→队列→切帧→Convert；校准 **rad** | [Velodyne](../sensor/lidar/velodyne.md) |
| `hesai` / `pandar` | XT32；校准 **deg** | [Hesai](../sensor/lidar/hesai.md) |
| `livox` | SDK1/SDK2；`model`/`sdk` 选型 | [Livox](../sensor/lidar/livox.md) |
| `rplidar` / `slamtec` | 2D 串口 SDK；A3 常 256000 | [RPLidar](../sensor/lidar/rplidar.md) |

| 约定 | 说明 |
|---|---|
| 点云 | `point_step=24`：`x,y,z,intensity,timestamp` |
| 回放 | `source_type: raw_packet` + `PushRawPacket` / `PushScan` |
| 补偿 | `enable_compensator` + 进程 `compensator.pose_channel` |
| 队列 | `lidar/packet_queue.hpp`：满丢最旧；online ReadLoop→ProcessLoop |
| 切帧 | `scan_cut.hpp`：`use_azimuth_cut` + `packets_per_scan` 上限 |

安装：`scripts/install_rplidar_sdk.sh`、`install_livox_sdk*.sh`。

## realsense

需 `AUTODRIVER_WITH_REALSENSE` + librealsense2 → `AUTODRIVER_HAVE_REALSENSE`。同机多流经 `camera/realsense/device_hub`。用 `model`/`serial`/`index` 选机，**勿**为型号新建 backend。

| 注册 | 说明 |
|---|---|
| `REGISTER_CAMERA_BACKEND(realsense, …)` | 图像 |
| `REGISTER_POINTCLOUD_BACKEND` | 深度点云 |
| `REGISTER_IMU_BACKEND(realsense, …)` | 板载 IMU（仅链入 SDK 时） |

`Publisher` 对每个 image channel 自动开 camera_info（`bridge/channels.hpp`）；帧级由 `has_camera_info` 决定是否写入。详见 [RealSense](../sensor/camera/realsense.md)。

## orbbec

需 `AUTODRIVER_WITH_ORBBEC` 且找到 OrbbecSDK。同机多流经 `camera/orbbec/device_hub`。无 SDK 时 Create→`nullptr`。

| `params` | 说明 |
|---|---|
| `stream` | `color` / `depth` / `left_ir` / `right_ir`（`ir`→left；`ir0` 单 IR） |
| `serial` / `index` / `model` | 选设备 |
| `width` / `height` / `fps` | `0` = SDK 默认档 |
| `frame_id` | 光学系覆盖 |
| `enable_laser` | IR 投影灯；Gemini 330 官方默认 **true** |
| `device_preset` | 官方默认 **`Default`** |
| `disparity_to_depth_mode` | **`HW`** / `SW` / `disable` |
| `enable_disparity_to_depth` | 后处理；官方 **true** |
| `enable_hardware_noise_removal_filter` | 官方 **false** |
| `enable_noise_removal_filter` | 软去噪；官方 **true** |
| `noise_removal_filter_min_diff` / `max_size` | 官方 **256** / **80** |
| `enable_spatial_filter` | 官方 **false** |

Channel 对齐 OrbbecSDK_ROS2 Gemini 330：`/camera/color|depth|left_ir|right_ir/image_raw`、点云 `/camera/depth/points` 或 `/camera/depth_registered/points`。参数文件：`config/camera/orbbec/gemini_330.yaml`。详见 [Orbbec](../sensor/camera/orbbec.md)。

## chassis（本体）

顶层 `chassis/`；与传感 Registry 同模式，消息为 automsgs vehicle_msgs + TwistStamped。

| backend | 状态 |
|---|---|
| `stub`（别名可 `sim`） | 差分积分，无硬件联调 |
| 厂商名 | `REGISTER_CHASSIS_BACKEND` + `chassis/<vendor>/` |

加厂商：实现 `ChassisDriver` → 宏注册 → YAML `chassis.backend`。详见包内 [`chassis/README.md`](../../../chassis/README.md) · [配置 · chassis](configuration.md#chassis本体)。

## radar / microphone / smartereye

| YAML | backend | 状态 | 落地方向 |
|---|---|---|---|
| `radar` | `conti` / `continental` | stub | ProtocolData + canbus |
| `microphone` | `respeaker` | stub | PortAudio / USB HID |
| `camera` | `smartereye` | stub | 厂商 SDK |

样本占位：`RadarSample`≈PointCloud2，`MicrophoneSample`≈Image（PCM）。扩展：实现 Create + 注册宏，**不必**改 Module。

## 扩展清单

| 模态 | 步骤 |
|---|---|
| 相机 / 点云 | `camera/<v>/` hub+driver → `REGISTER_CAMERA_*` / `POINTCLOUD_*` → CMake（可选 `WITH_*`）→ `config/camera/<v>/` |
| 3D 激光 | `lidar/<v>/` → `REGISTER_LIDAR_BACKEND` → `point_step=24` → `config/lidar/<v>/` |
| 2D 激光 | `REGISTER_LIDAR2D_BACKEND` |
| IMU / GPS | `REGISTER_IMU_*` / `GPS_*`；串口用 `Stream`；新协议加 `GnssParser` |
| 底盘 | `chassis/<v>/` → `REGISTER_CHASSIS_BACKEND` |
| Radar / Mic | 先占名 stub，再换真 Create |

**勿改** `*Module` / `SensorManager` / `ChassisManager` 编排语义（除非改编排本身）。

## 外置插件（高级）

YAML `library` 非空时从 `plugin_dir` / `AUTODRIVER_PLUGIN_DIR` 加载 `.so`（须导出同名 `SensorModule`）。常规部署：`library` 留空，用内置 modules。
