# 常见问题

## 找不到配置文件

```
failed to load autodriver config: ... (optional override: export AUTODRIVER_PATH=<config parent>)
```

确认：

1. `AUTODRIVER_PATH` 指向**含 `config/` 的目录**（不是 `config` 本身）
2. 默认文件名为 `autodriver_hardware.yaml`
3. 安装树下检查 `share/autodriver/config/`

仓库根目录开发时：

```bash
export AUTODRIVER_PATH=$PWD/autodriver
```

## 没有传感器被加载

- typed 组里只有 `enable: true` 的条目会进入 `Config`
- 全为 `false` 时日志会有 `no enabled sensors`
- `enable: false` 的设备**不能**再 `Attach`；热插拔也只作用于已加载且带 `match` 的条目

## 插件 / 类创建失败

常规路径下模块已编入 `libautodriver.so`，**不需要** `libautodriver_imu.so` 之类分模态库。

若日志提示 CreateClassObj 失败：

1. 确认链接的是当前构建的 `libautodriver`
2. `module` 类名与注册名一致（如 `ImuModule`、`CameraModule`、`PointCloudModule`）
3. 仅当 YAML 写了 `library` 时才需要外置 `.so`，并把目录放进 `plugin_dir` 或 `LD_LIBRARY_PATH`

## RealSense 不可用

构建日志中若无 `librealsense2 ... enabled`，则 RealSense backend 未编入。安装 Intel librealsense2 后重新配置，或显式 `-DAUTODRIVER_WITH_REALSENSE=OFF`。

Orbbec 同理：需构建日志出现 `OrbbecSDK enabled`；否则 `Create("orbbec")` 返回空。安装 OrbbecSDK 后 `-DAUTODRIVER_WITH_ORBBEC=ON` 重新配置。

多台设备时在 `params.serial`（或 `index`+`model`）上区分；同机多流共用一个 hub。

## udev 热插拔不生效

- 仅 Linux + 链接到 `libudev`（`AUTODRIVER_HAVE_UDEV`）
- `hotplug.enable_udev` 须为 `true`
- 传感器须已 `enable: true` 且 `match` 非空（serial 的 `port` 会自动补 match）

## CAN / 串口权限

访问 `/dev/ttyUSB*`、`can0` 通常需要 dialout 组或相应 udev 规则。CAN 需先 `ip link set can0 up type can ...`。

## 与 Autolink 的关系

Autodriver **采集**；**发布**经 `bridge::Publisher`（同一 `libautodriver`）写 Autolink。运行 `autodriver` 前设置好 `AUTOLINK_PATH` 与 Autolink 运行时，否则 `Publisher::Initialize` 可能失败。

`LD_LIBRARY_PATH` 需包含 `build/lib`，以便加载 `libautodriver` 与 Autolink 依赖（不是为了加载分模态传感器插件）。

## Lidar / Range 无数据

- **3D Velodyne / Hesai**：`backend: velodyne` 或 `hesai`，确认 `data_port` 可达且防火墙放行 UDP  
- **2D / Range**：仍为 attach-only 骨架  
- RealSense 点云：`point_cloud` + `PointCloudModule`

## Radar / Microphone / SmarterEye 无数据

这些是 **stub**：工厂已注册，但 `Create` 返回 `nullptr`，Attach 会失败或跳过采集。

| YAML | Module | backend | 待补齐 |
|---|---|---|---|
| `radar` | `RadarModule` | `conti` | Conti ProtocolData + `canbus` |
| `microphone` | `MicrophoneModule` | `respeaker` | PortAudio / USB |
| `camera` + `backend: smartereye` | `CameraModule` | `smartereye` | 厂商 SDK |

单测仅校验注册表 / FakeCan：`test_skeleton_modules`、`test_canbus_skeleton`。

## GNSS Parser 怎么用

```cpp
#include "autodriver/gps/parser/parser.hpp"

auto parser = autodriver::gps::GnssParserRegistry::Instance().Create("nmea");
auto fix = parser->Consume(bytes, n);  // 可选 ParsedFix
```

内置名：`nmea`、`nmea0183`。串口 GPS 驱动仍可直接调 `ParseGgaSentence`；工厂便于后续厂商报文。

## 诊断话题

`SensorManager::ReportDiagnostic` 在 Attach/Detach 成功或失败时回调 `SampleSink::OnDiagnostic`。  
`bridge::Publisher` 懒创建 Writer，发布 `diagnostic_msgs/DiagnosticArray` 到 `/diagnostics`（可用 `SetDiagnosticsChannel` 覆盖）。

## 源码在哪

| 路径 | 内容 |
|---|---|
| `autodriver/common/` | Stream、串口、CanSocket、外参 YAML、设备状态 |
| `autodriver/canbus/` | ProtocolData、Receiver、Client、Sender、byte |
| `autodriver/gps/parser/` | `GnssParserRegistry`（NMEA） |
| `autodriver/camera/` | `realsense/`、`orbbec/`、backend 注册表 |
| `autodriver/radar/` / `microphone/` / `smartereye/` | stub 模态 |
| `autodriver/lidar/` | Lidar 基类、`packet_queue`、`scan_cut`、MotionCompensator、Velodyne/Hesai、厂商 stub |

## 文档构建

```bash
pip install -r autodriver/docs/requirements.txt
cmake --build build --target docs
```

需系统/环境可执行 `mkdocs`；未找到时 CMake 会禁用 `docs` 目标并打印提示。本地预览：

```bash
cd autodriver/docs && mkdocs serve
```
