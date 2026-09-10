# API 概览

头文件均在 `autodriver/` 下；链接目标 `autodriver`（`libautodriver.so`）；进程入口产物 `autodriver`（CMake target：`autodriver_main`）。

设计摘要见 [架构](../guide/architecture.md)；嵌入用法见 [使用方式](../guide/usage.md)。

## 类型关系

```
Config
  └─ Sensor[]
DeviceMatch
SensorManager
  ├─ SensorModule / SensorPlugin
  ├─ BackendRegistry → SensorDriver
  ├─ common::Stream × Parser（serial GPS/IMU）
  ├─ lidar::LidarComponentBase（厂商扩展）
  ├─ SensorHub
  └─ SampleSink → bridge::Publisher
```

源码按模态分包：`common/`、`canbus/`、`imu/`、`gps/`、`camera/`、`lidar/`、`radar/`、`microphone/`、`smartereye/`、`chassis/`、`bridge/`。

## 注册表 API（模块化扩展点）

厂商驱动通过静态宏注册，Module 只查表：

```cpp
// 3D lidar
#include "autodriver/lidar/backend_register.hpp"
REGISTER_LIDAR_BACKEND(myvendor, "myvendor", CreateMyDriver, "alias");

// 2D lidar
#include "autodriver/lidar/lidar_2d_backend_register.hpp"
REGISTER_LIDAR2D_BACKEND(foo, "foo", CreateFooDriver);

// 相机 / 点云
#include "autodriver/camera/backend_register.hpp"
REGISTER_CAMERA_BACKEND(tag, "backend", CreateFn);
REGISTER_POINTCLOUD_BACKEND(tag, "backend", CreateFn);

// 本体 / 底盘（与 autonomy/vehicle 解耦）
#include "autodriver/chassis/backend_register.hpp"
REGISTER_CHASSIS_BACKEND(mybot, "mybot", CreateMyBotDriver);
```

工厂签名统一为：

```cpp
std::shared_ptr<SensorDriver> CreateXxx(
    const SensorId& id, const hardware::DriverParams& params);
```

运行时：

```cpp
auto drv = lidar::LidarBackendRegistry::Instance().Create(
    "velodyne", id, params);
```

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

## SensorDriver

```cpp
#include "autodriver/sensor_driver.hpp"

class SensorDriver {
  using SampleCallback = std::function<void(std::unique_ptr<SensorSample>)>;
  virtual SensorType GetType() const = 0;
  virtual const SensorId& GetSensorId() const = 0;
  virtual bool Start() = 0;
  virtual void Stop() = 0;
  virtual bool IsRunning() const = 0;
  virtual void SetSampleCallback(SampleCallback) = 0;
};
```

`DriverParams` = `unordered_map<string,string>`；辅助：`GetString`/`ParseInt`/`ParseBool`/`ParseDouble`（`driver_params.hpp`）。

3D：`LidarComponentBase`（`WritePointCloud` 纯虚；`InitPacket`/`InjectScan` 可选）+ 可选 `MotionPoseSink`（`PushPose`/`SetPoseLookup`/`pose_buffer`）。Velodyne/Hesai 另有 `PushRawPacket`/`PushScan`（非虚，回放）。

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
| `SetSink` | 注册 `SampleSink` |
| `Initialize` | id 查重；失败 false |
| `Start` / `Stop` | 启停 Hub/udev/autostart Attach |
| `Attach` / `Detach` | 幂等；失败 false 不崩进程 |
| `HandleDeviceEvent` | udev 或测试注入 |
| `hub()` | `SensorHub&` |
| `PushLidarPose` / `SetLidarPoseLookup` | 运动补偿灌姿 |
| `ReportDiagnostic` | → sink `OnDiagnostic` |

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
| 头文件 | 内容 |
|---|---|
| `config_loader.hpp` | YAML 加载 |
| `sensor_manager.hpp` | 编排；`PushLidarPose` / `SetLidarPoseLookup` |
| `sensor_hub.hpp` | 对齐 |
| `sensor_module.hpp` / `sensor_plugin.hpp` | 插件 |
| `sample_sink.hpp` / `bridge/publisher.hpp` | 发布 |
| `bridge/pose_feeder.hpp` | Odometry → `PushLidarPose` |
| `common/stream.hpp` | Serial/UDP + Reconnect |
| `common/calibration.hpp` | 外参 YAML |
| `common/status.hpp` | 健康；→ `/diagnostics` |
| `camera/backend_registry.hpp` + `backend_register.hpp` | 相机/点云 Registry |
| `camera/realsense/` / `camera/orbbec/` | 真驱动（需 SDK） |
| `lidar/backend_registry.hpp` + `backend_register.hpp` | 3D Registry / 宏 |
| `lidar/lidar_2d_backend_*.hpp` | 2D Registry / 宏 |
| `lidar/livox/` / `lidar/rplidar/` / `lidar/hesai/` / `lidar/velodyne/` | 厂商 |
| `lidar/motion_pose_sink.hpp` / `motion_compensator.hpp` | 补偿 |
| `lidar/packet_queue.hpp` / `scan_cut.hpp` | 收包 / 切帧 |
| `canbus/` | Client/Sender/Receiver/ProtocolData |
| `gps/parser/parser.hpp` | NMEA 工厂 |
| `radar/` `microphone/` `smartereye/` | stub |
| `sensor_traits.hpp` | 默认 channel |
| `types/sensor_*.hpp` | 模态与样本 |

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
