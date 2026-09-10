# 常见问题

## 配置路径

```
failed to load autodriver config: ... export AUTODRIVER_PATH=<config parent>
```

1. `AUTODRIVER_PATH` = **含 `config/` 的包根**（不是 `config` 目录本身）  
2. 默认 basename：`autodriver_hardware.yaml`  
3. 回退：`$AUTODRIVER_DISTRIBUTION_HOME/share/autodriver/config/`  

开发示例：`export AUTODRIVER_PATH=$PWD/src/autonomy/autodriver`（按仓库布局调整）。

## 无传感器

- 仅 `enable: true`（或 `attach_on_start: true`）进入 Config  
- 全 false → 日志 `no enabled sensors`  
- `enable: false` **不能**再 Attach；udev 也只作用于已加载且 `match` 有效的条目  

## 类 / 插件加载失败

模块编在 `libautodriver.so`，无需分模态 `.so`。  
排查：链接的是否为当前构建；`module` 名是否为 `ImuModule`/`CameraModule`/…；仅 YAML `library:` 非空才加载外置库（`plugin_dir` / `LD_LIBRARY_PATH`）。

## RealSense / Orbbec

构建 STATUS 须出现 `librealsense2 … enabled` / `OrbbecSDK enabled`。否则 Create 空。多机用 `params.serial` 或 `index`+`model`；同机多流共用 hub（折叠配置）。

## RPLidar / Livox

| 问题 | 处理 |
|---|---|
| RPLidar Create 空 | `./scripts/install_rplidar_sdk.sh`；CMake 找 `RplidarSDK` |
| 连不上 A3 | `params_file: lidar/slamtec/a3.yaml`（256000） |
| Livox Create 空 | `install_livox_sdk2.sh` 或 `install_livox_sdk.sh` |
| Mid-360 无数据 | `host_ip`/`lidar_ip` 同网段；端口与官方 JSON 一致 |

## udev

需 Linux + `libudev`（`AUTODRIVER_HAVE_UDEV`）+ `hotplug.enable_udev: true` + 已 enable 且 `match` 非空。serial 的 `port` 会自动补 `match.subsystem=tty`。

## 串口 / CAN

`dialout` 组；RPLidar 可用 `create_udev_rules.sh` → `/dev/rplidar`。CAN：`ip link set can0 up type can bitrate …`。

## Autolink

采集在 autodriver；发布经 `Publisher`。`Publisher::Initialize` 失败查 Autolink 运行时与 `AUTOLINK_PATH`。`LD_LIBRARY_PATH` 含 `build/lib`。

## Lidar / 点云无数据

| backend | 查 |
|---|---|
| `velodyne` / `hesai` | UDP `data_port`、防火墙、`model`/校准 |
| `livox` | SDK、网段、JSON/`host_ip` |
| `rplidar` | 串口、波特率、SDK |
| RealSense/Orbbec 点云 | `point_clouds` 或扁平 `point_cloud` enable |

## Stub（无真数据）

| YAML | backend | 待补 |
|---|---|---|
| `radar` | `conti` | ProtocolData + canbus |
| `microphone` | `respeaker` | PortAudio |
| `camera` + smartereye | `smartereye` | 厂商 SDK |
| `rslidar`/`lslidar`/… | stub | 仿 Velodyne/Livox 实现 |

单测：`test_skeleton_modules`、`test_canbus_skeleton`。

## GNSS Parser

```cpp
auto p = autodriver::gps::GnssParserRegistry::Instance().CreateParser("nmea");
p->Consume(bytes, n);
```

别名：`nmea0183`。

## 诊断

Attach/Detach 成败 → `SampleSink::HandleDiagnostic` → Publisher → `/diagnostics`（可 `SetDiagnosticsChannel`）。

## 源码索引

| 路径 | 内容 |
|---|---|
| `common/` | Stream、`SerialByteDriverBase`、`CanSensorDriverBase`、`BackendRegistry`、串口、UDP、外参、status |
| `canbus/` | ProtocolData、Receiver、Client、Sender |
| `camera/` | realsense、orbbec、registry |
| `lidar/` | `UdpScanDriverBase`、Livox assembler 基类、velodyne/hesai、rplidar、queue、compensator、stubs |
| `imu/` `gps/` | serial/CAN 驱动（CRTP 基类）+ registry |
| `gps/parser/` | NMEA 工厂 |
| `bridge/` | Publisher、PoseFeeder |
| `config/` | 硬件 YAML + 厂商 params |
| `scripts/` | SDK / udev |

## 文档构建

```bash
pip install -r docs/requirements.txt
cd docs && mkdocs serve
# 或 cmake --build build --target docs
```
