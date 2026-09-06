# API 概览

头文件均在 `autodriver/` 下；链接目标 `autodriver`（`libautodriver.so`）；进程入口产物 `autodriver`（CMake target：`autodriver_main`）。

## 类型关系

```
Config
  └─ Sensor[]
DeviceMatch
SensorManager
  ├─ SensorModule / SensorPlugin
  ├─ common::Stream × Parser（serial GPS/IMU）
  ├─ lidar::LidarComponentBase（厂商扩展）
  ├─ SensorHub
  └─ SampleSink → bridge::Publisher
```

源码按模态分包：`common/`、`canbus/`、`imu/`、`gps/`、`camera/`、`lidar/`、`radar/`、`microphone/`、`smartereye/`、`bridge/`。

## Config

```cpp
#include "autodriver/config.hpp"
#include "autodriver/config_loader.hpp"

autodriver::Config config = autodriver::LoadConfig();
// 或
config = autodriver::LoadConfig(dir, autodriver::kDefaultConfigBasename);
```

| 成员 | 含义 |
|---|---|
| `node_name` | 传给 `Publisher` 的 Autolink 节点名 |
| `plugins` | 外置插件目录（YAML `plugin_dir`） |
| `hotplug.udev` | 是否开 udev |
| `alignment` | Hub 开关与 `SensorHub::Options` |
| `sensors` | `Config::Sensor` 列表 |
| `HasDuplicateId()` | id 查重 |
| `FindId(DeviceMatch)` | udev 匹配 |

`Config::Sensor`：`module`、`library`、`id`、`channels`、`backend`、`autostart`、`match`、`params`。

手写示例见 `examples/demo_main.cpp`：

```cpp
autodriver::Config config;
autodriver::Config::Sensor lidar2d;
lidar2d.module = "Lidar2dModule";
lidar2d.id = "lidar/front";
lidar2d.autostart = true;
config.sensors = {lidar2d};
```

## SensorManager

```cpp
#include "autodriver/sensor_manager.hpp"
#include "autodriver/bridge/publisher.hpp"

autodriver::bridge::Publisher publisher(config.node_name);
publisher.Initialize();

autodriver::SensorManager manager(std::move(config));
manager.SetSink(&publisher);
manager.Initialize();
manager.Start();
// manager.Attach("imu/torso");
// manager.Detach("imu/torso");
manager.Stop();
```

| 方法 | 说明 |
|---|---|
| `SetSink` | 注册 `SampleSink`（发布） |
| `Initialize` | 查重；失败返回 false |
| `Start` / `Stop` | 生命周期 |
| `Attach` / `Detach` | 幂等 |
| `HandleDeviceEvent` | udev 或测试注入 |
| `hub()` | `SensorHub` 引用 |

## SensorHub

可选对齐旁路。`alignment.enable` 时由 Manager 启动。选项：`alignment_window`、`publish_period`、`buffer_capacity`。样本以 `shared_ptr` 入缓冲，无 protobuf Clone。

## SensorModule / SensorPlugin

- `SensorModule`：插件接口（`Init` / `Start` / `Stop`）
- `SensorPlugin<kType, kCapture>`：`kCapture=true` 建驱动并回调；`false` 为 attach-only
- `Context`：`node`（可空）、`sensor`（`Config::Sensor`）、`hook`（对齐旁路）

内置类名：`ImuModule`、`GpsModule`、`CameraModule`、`PointCloudModule`、`Lidar2dModule`、`Lidar3dModule`、`LidarModule`、`RadarModule`、`MicrophoneModule`、`RangeModule`。

## SampleSink / Publisher

```cpp
#include "autodriver/sample_sink.hpp"
#include "autodriver/bridge/publisher.hpp"
```

| 接口 | 时机 |
|---|---|
| `OnAttach(sensor, type)` | 开 Writer（相机含 camera_info） |
| `OnDetach(id)` | 关 Writer |
| `OnSample(sample)` | 写 Autolink |

`bridge::Publisher` 拥有 Autolink `Node`；核心采集路径不直接调用 `Write`。通道解析见 `ResolveChannel`（`sensor_traits.hpp`）。

## DeviceMatch

```cpp
bool MatchDevice(const DeviceMatch& observed, const DeviceMatch& rule);
```

字段：`subsystem`、`device`、`vendor`、`product`、`serial`。

## 通道辅助

- `ResolveChannel(channel, id, type, stream)` — 默认话题
- `bridge::CameraInfoChannelForImage(image_channel)` — 由图像话题推导 camera_info

## 相关头文件

| 头文件 | 内容 |
|---|---|
| `config_loader.hpp` | YAML 配置加载 |
| `common/status.hpp` | 健康快照；经 Publisher 发到 `/diagnostics` |
| `sensor_manager.hpp` | 编排；`PushLidarPose` / `SetLidarPoseLookup` |
| `bridge/pose_feeder.hpp` | Odometry → `PushLidarPose` |
| `lidar/motion_pose_sink.hpp` | 补偿位姿灌入接口 |
| `lidar/hesai/calibration.hpp` | XT32 仰角 YAML |
| `sensor_hub.hpp` | 对齐 |
| `sensor_module.hpp` / `sensor_plugin.hpp` | 插件 |
| `sample_sink.hpp` / `bridge/publisher.hpp` | 发布 |
| `common/stream.hpp` | 传输抽象（Serial / UDP + Reconnect） |
| `common/calibration.hpp` | 外参 YAML → `Extrinsic` |
| `camera/backend_registry.hpp` | camera / point_cloud 厂商工厂注册表 |
| `camera/realsense/camera_driver.hpp` | RealSense 图像驱动 |
| `camera/orbbec/` | Orbbec hub + camera / pointcloud（需 OrbbecSDK） |
| `lidar/backend_registry.hpp` | lidar_3d 厂商工厂注册表 |
| `lidar/backend_register.hpp` | `REGISTER_LIDAR_BACKEND` 宏 |
| `lidar/motion_compensator.hpp` | 扫面内运动补偿（需 PoseLookup） |
| `lidar/hesai/udp_driver.hpp` | Hesai XT32 UDP → Convert → PointCloud2 |
| `canbus/` | CAN Client/Sender/Receiver/ProtocolData/byte |
| `radar/` | Conti stub + Registry |
| `microphone/` | Respeaker stub + Registry |
| `smartereye/` | Camera backend stub |
| `gps/parser/parser.hpp` | GNSS Parser 工厂（NMEA；serial GPS 已接入） |
| `lidar/packet_queue.hpp` | 有界收包队列 |
| `common/status.hpp` | 设备健康枚举 |
| `sensor_traits.hpp` | 类型 traits、默认 channel |
| `types/sensor_type.hpp` / `types/sensor_sample.hpp` | 模态与样本 |

## canbus

```cpp
#include "autodriver/canbus/byte.hpp"
#include "autodriver/canbus/can_client.hpp"
#include "autodriver/canbus/can_receiver.hpp"
#include "autodriver/canbus/protocol_data.hpp"
```

| 类型 | 用途 |
|---|---|
| `Byte` | 单字节位域 set/get |
| `ProtocolData<T>` / `MessageManager<T>` | CAN id → Parse → publish |
| `CanReceiver<T>` | SocketCAN 收线程 + dispatch |
| `CanClient` | `SocketCanClient` / `FakeCanClient`（`CreateCanClient`） |
| `CanSender` | 周期 TX |

## GNSS Parser

```cpp
#include "autodriver/gps/parser/parser.hpp"

auto p = autodriver::gps::GnssParserRegistry::Instance().Create("nmea");
```

`Consume` 流式喂字节；内置 `nmea` / `nmea0183` → `Nmea0183Parser`。

## Lidar PacketQueue

```cpp
#include "autodriver/lidar/packet_queue.hpp"

autodriver::lidar::PacketQueue<PacketBuffer> q(256);
q.Push(std::move(pkt));           // 满则丢最旧
auto item = q.TryPop();           // 空则 nullopt
```

Velodyne / Hesai **online** 路径已接入（ReadLoop → queue → ProcessLoop）。  
切帧：`lidar/scan_cut.hpp`（`use_azimuth_cut` + `packets_per_scan` 上限）。
