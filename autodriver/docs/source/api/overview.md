# API 概览

头文件均位于 `autodriver/` 下；链接目标为 `autodriver`（`libautodriver.so`）；进程入口产物为 `autodriver`（CMake target：`autodriver_main`）。

设计摘要见 [架构](../guide/architecture.md)；嵌入用法见 [使用方式](../guide/usage.md)；术语见 [术语](../guide/glossary.md)。

> 接口语义以头文件与实现为准。线程安全性若未在源码注释或正文中写明，视为「未在当前源码中明确」。本文不构成运行时验证报告。

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

源码按模态分包：传感位于 `autodriver/{common,canbus,imu,gps,camera,lidar,radar,microphone,smartereye,bridge}/`；本体位于顶层 `chassis/`。

## 注册表 API（模块化扩展点）

厂商驱动通过静态宏注册，Module 仅查表创建：

```cpp
// IMU / GPS
#include "autodriver/imu/backend_register.hpp"
REGISTER_IMU_BACKEND(serial, "serial", CreateSerialImuDriver);
#include "autodriver/gps/backend_register.hpp"
REGISTER_GPS_BACKEND(serial, "serial", CreateSerialGpsDriver);

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

// 本体 / 底盘（与 autonomy/vehicle 解耦；源码在顶层 chassis/）
#include "chassis/backend_register.hpp"
REGISTER_CHASSIS_BACKEND(mybot, "mybot", CreateMyBotDriver);
```

Creator（`REGISTER_*` / `CreateXxx`）统一返回 owning raw 指针：

```cpp
SensorDriver* CreateXxx(
    const SensorId& id, const hardware::DriverParams& params);
// chassis：ChassisDriver* CreateXxx(const ChassisId&, const DriverParams&);
```

Registry 在运行时再封装为 `SharedPtr`（`NamedProductFactory` / `autolink::common::Factory`）：

```cpp
auto drv = lidar::LidarBackendRegistry::Instance().CreateDriver(
    "velodyne", id, params);  // → SensorDriver::SharedPtr
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
| `hotplug.udev` | 是否启用 udev |
| `alignment` | Hub 开关与 `SensorHub::Options` |
| `sensors` | `Config::Sensor` 列表 |
| `HasDuplicateId()` | 检测 id 是否重复 |
| `FindId(DeviceMatch)` | 按 udev 规则匹配传感器 id |

`Config::Sensor` 主要字段：`module`、`library`、`id`、`channels`、`backend`、`autostart`、`match`、`params`。

程序内构造示例见 `examples/demo_main.cpp`：

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
  virtual SensorType GetSensorType() const = 0;
  virtual const SensorId& GetSensorId() const = 0;
  virtual bool Start() = 0;
  virtual void Stop() = 0;
  virtual bool IsRunning() const = 0;
  virtual void SetSampleCallback(SampleCallback) = 0;
};
```

`DriverParams` 为 `unordered_map<string,string>`；辅助函数：`GetString` / `ParseInt` / `ParseBool` / `ParseDouble`（`driver_params.hpp`）。

**所有权与线程（源码确认）**：`SetSampleCallback` 收到的 `unique_ptr<SensorSample>` 在**驱动采集线程**上交付；调用方不得假设可与 `Stop()` 无同步地并发调用（跨线程规则未在接口注释中完整保证时，按「未在当前源码中明确」处理）。Creator 返回 owning raw；经 Registry 后由 `SharedPtr` 持有。

3D 激光：`LidarComponentBase`（`WritePointCloud` 纯虚；`InitPacket` / `InjectScan` 可选）+ 可选 `MotionPoseSink`（`PushPose` / `SetPoseLookup` / `pose_buffer`）。Velodyne / Hesai 另提供非虚的 `PushRawPacket` / `PushScan`，用于回放。

## SensorManager

```cpp
#include "autodriver/sensor_manager.hpp"
#include "autodriver/bridge/publisher.hpp"

autodriver::bridge::Publisher publisher(config.node_name);
publisher.Initialize();

autodriver::SensorManager manager(std::move(config));
manager.SetSampleSink(&publisher);
manager.Initialize();
manager.Start();
// manager.AttachSensor("imu/torso");
// manager.DetachSensor("imu/torso");
manager.Stop();
```

| 方法 | 说明 |
|---|---|
| `SetSampleSink` | 注册 `SampleSink` |
| `Initialize` | 检测 id 重复；失败返回 `false` |
| `Start` / `Stop` | 启停 Hub、udev 与 autostart Attach |
| `AttachSensor` / `DetachSensor` | 幂等；失败返回 `false`，不终止进程 |
| `HandleDeviceEvent` | udev 事件或测试注入 |
| `GetHub()` | 返回 `SensorHub&` |
| `PushLidarPose` / `SetLidarPoseLookup` | 向运动补偿注入位姿 |
| `ReportDiagnostic` | 转发至 sink 的 `HandleDiagnostic` |

## SensorHub

可选**对齐旁路**（见 [术语](../guide/glossary.md)）。当 `alignment.enable` 为真时由 Manager 启动。选项包括：`alignment_window`、`publish_period`、`buffer_capacity`。样本以 `shared_ptr` 入缓冲，不依赖 protobuf Clone。

## SensorModule / SensorPlugin

- `SensorModule`：插件接口（`Init` / `Start` / `Stop`）
- `SensorPlugin<kType, kCapture>`：`kCapture=true` 时创建驱动并回调；`false` 时为仅 Attach（无采集）
- `Context`：`node`（可为空）、`sensor`（`Config::Sensor`）、`hook`（对齐旁路）

内置类名：`ImuModule`、`GpsModule`、`CameraModule`、`PointCloudModule`、`Lidar2dModule`、`Lidar3dModule`、`LidarModule`、`RadarModule`、`MicrophoneModule`、`RangeModule`。

样本类型（与代码一致）：`ImuSample`、`GpsSample`、`CameraFrame`、`LidarScan`、`LidarCloud`、`LidarPacketScan`、`RadarSample`、`MicrophoneSample`。

## SampleSink / Publisher

```cpp
#include "autodriver/sample_sink.hpp"
#include "autodriver/bridge/publisher.hpp"
```

| 接口 | 时机 |
|---|---|
| `HandleSensorAttach(sensor, type)` | 打开 Writer（相机含 camera_info） |
| `HandleSensorDetach(id)` | 关闭 Writer |
| `HandleSensorSample(sample)` | 写入 Autolink |

`bridge::Publisher` 拥有 Autolink `Node`；核心采集路径不直接调用 `Write`。通道解析参见 `ResolveChannel`（`sensor_traits.hpp`）。

## DeviceMatch

```cpp
bool MatchDevice(const DeviceMatch& observed, const DeviceMatch& rule);
```

字段：`subsystem`、`device`、`vendor`、`product`、`serial`。

## 通道辅助

- `ResolveChannel(channel, id, type, stream)` — 默认话题
- `bridge::CameraInfoChannelForImage`（`bridge/channels.hpp`）— 由图像话题推导 camera_info

## 相关头文件

| 头文件 | 内容 |
|---|---|
| `config_loader.hpp` | YAML 加载 |
| `sensor_manager.hpp` | 编排；`PushLidarPose` / `SetLidarPoseLookup` |
| `sensor_hub.hpp` | 时间对齐 |
| `sensor_module.hpp` / `sensor_plugin.hpp` | 插件 |
| `sample_sink.hpp` / `bridge/publisher.hpp` | 发布 |
| `bridge/pose_feeder.hpp` | Odometry → `PushLidarPose` |
| `common/stream.hpp` | Serial/UDP 与重连 |
| `common/serial_byte_driver_base.hpp` | 串口读环 CRTP（IMU/GPS serial） |
| `common/can_sensor_driver_base.hpp` | CAN `CanReceiver` 包装 CRTP |
| `common/backend_registry.hpp` | 模态 Registry 模板（默认 backend 与 alias） |
| `common/calibration.hpp` | 外参 YAML |
| `common/status.hpp` | 健康状态；发布至 `/diagnostics` |
| `common/named_factory.hpp` | `NamedProductFactory` → autolink Factory |
| `imu/backend_registry.hpp` + `backend_register.hpp` | IMU Registry / 宏 |
| `gps/backend_registry.hpp` + `backend_register.hpp` | GPS Registry / 宏 |
| `gps/parser/parser.hpp` | NMEA `GnssParserRegistry` |
| `camera/backend_registry.hpp` + `backend_register.hpp` | 相机/点云 Registry |
| `camera/realsense/` / `camera/orbbec/` | 已实现驱动（依赖对应 SDK） |
| `lidar/backend_registry.hpp` + `backend_register.hpp` | 3D Registry / 宏 |
| `lidar/lidar_2d_backend_*.hpp` | 2D Registry / 宏 |
| `lidar/livox/` / `lidar/rplidar/` / `lidar/hesai/` / `lidar/velodyne/` | 厂商实现 |
| `lidar/motion_pose_sink.hpp` / `motion_compensator.hpp` | 运动补偿 |
| `lidar/packet_queue.hpp` / `scan_cut.hpp` | 收包 / 切帧 |
| `canbus/` | Client / Sender / Receiver / ProtocolData |
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
| `CanReceiver<T>` | SocketCAN 接收线程与分发 |
| `CanClient` | `SocketCanClient` / `FakeCanClient`（`CreateCanClient`） |
| `CanSender` | 周期发送 |

## GNSS Parser

```cpp
#include "autodriver/gps/parser/parser.hpp"

auto p = autodriver::gps::GnssParserRegistry::Instance().CreateParser("nmea");
```

`Consume` 以流式方式喂入字节；内置 `nmea` / `nmea0183` 映射至 `Nmea0183Parser`。

## Lidar PacketQueue

```cpp
#include "autodriver/lidar/packet_queue.hpp"

autodriver::lidar::PacketQueue<PacketBuffer> q(256);
q.Push(std::move(pkt));           // 队列满时丢弃最旧项
auto item = q.TryPop();           // 空队列返回 nullopt
```

Velodyne / Hesai 的 **online** 路径已接入（ReadLoop → queue → ProcessLoop）。  
切帧参见 `lidar/scan_cut.hpp`（`use_azimuth_cut` 与 `packets_per_scan` 上限）。
