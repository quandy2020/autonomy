# 常见问题

> 下列条目为源码/配置对照下的排障指引；实机结果取决于 SDK、权限、网段与硬件，不视为已完成运行验证。

## 配置路径

```
failed to load autodriver config: ... export AUTODRIVER_PATH=<config parent>
```

1. `AUTODRIVER_PATH` 须指向**含 `config/` 的包根**（而非 `config` 目录本身）。  
2. 默认配置文件名为 `autodriver_hardware.yaml`。  
3. 回退路径：`$AUTODRIVER_DISTRIBUTION_HOME/share/autodriver/config/`。  

开发示例：`export AUTODRIVER_PATH=$PWD/src/autonomy/autodriver`（按仓库布局调整）。

## `undefined symbol: …ChassisManager::Start…`

`LD_LIBRARY_PATH` 加载了**旧**的 `libautodriver.so`（不含 chassis）。colcon 嵌套构建产物在：

```bash
export LD_LIBRARY_PATH=$PWD/build/autonomy/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/autonomy/bin:$PATH
```

不要优先使用过期的 `build/lib/libautodriver.so`。可用 `nm -D …/libautodriver.so | c++filt | grep ChassisManager::Start` 确认符号存在。

## launch 找不到 `autodriver.launch`

```bash
export AUTOLINK_LAUNCH_PATH=$PWD/src/autonomy/autodriver/launch
autolink launch start autodriver.launch
```

## 无传感器输出

- 仅 `enable: true`（或旧别名 `attach_on_start: true`）的条目进入 `Config`。  
- 全部为 false 时，日志输出 `no enabled sensors`。  
- `enable: false` 的条目**不可**再 Attach；udev 仅作用于已加载且 `match` 有效的条目。  

## 类 / 插件加载失败

模态 Module 编译进 `libautodriver.so`，无需分模态独立共享库。  
排查要点：是否链接当前构建产物；`module` 名是否为 `ImuModule`、`CameraModule` 等；仅当 YAML `library:` 非空时才加载外置库（`plugin_dir` / `LD_LIBRARY_PATH`）。

## RealSense / Orbbec

构建 STATUS 须出现 `librealsense2 … enabled` / `OrbbecSDK enabled`；否则 Create 返回 `nullptr`。多设备时使用 `params.serial` 或 `index` 与 `model`；同机多流共享 device hub（折叠配置）。

若日志出现 `module Start failed: camera/realsense_*`，先看紧随其后的  
`RealSense camera start failed: …`（例如 `Device or resource busy`）：

- **同一时刻只能有一个** `autodriver` 打开 D455（不要同时跑 `autodriver` 与 `autolink launch start autodriver.launch`）。
- 先 `pkill -f autodriver`，确认无残留进程后再启动。
- USB 带宽不足时可关掉 `streams` 里不需要的 IR / points，或降低 `width`/`fps`。

## RPLidar / Livox

| 问题 | 处理 |
|---|---|
| RPLidar Create 返回 `nullptr` | 执行 `./scripts/install_rplidar_sdk.sh`；确认 CMake 找到 `RplidarSDK` |
| 无法连接 A3 | 使用 `params_file: lidar/slamtec/a3.yaml`（波特率 256000） |
| Livox Create 返回 `nullptr` | 执行 `install_livox_sdk2.sh` 或 `install_livox_sdk.sh` |
| Mid-360 无数据 | 确认 `host_ip`/`lidar_ip` 处于同一网段；端口与官方 JSON 一致 |

## udev

须同时满足：Linux、`libudev`（`AUTODRIVER_HAVE_UDEV`）、`hotplug.enable_udev: true`、传感器已 enable 且 `match` 非空。serial 条目若已写 `port`，可自动补全 `match.subsystem=tty`。

## 串口 / CAN

用户须加入 `dialout` 组。RPLidar 可选用 `create_udev_rules.sh` 生成 `/dev/rplidar`。CAN：`ip link set can0 up type can bitrate …`。

## Autolink

采集在 autodriver 进程内完成；发布经 `Publisher`。若 `Publisher::Initialize` 失败，检查 Autolink 运行时与 `AUTOLINK_PATH`；确保 `LD_LIBRARY_PATH` 包含 `build/lib`。

## Lidar / 点云无数据

| backend | 排查项 |
|---|---|
| `velodyne` / `hesai` | UDP `data_port`、防火墙、`model` 与校准文件 |
| `livox` | SDK、网段、JSON / `host_ip` |
| `rplidar` | 串口、波特率、SDK |
| RealSense / Orbbec 点云 | `point_clouds` 或扁平 `point_cloud` 是否 enable |

## Stub（无真实采集数据）

| YAML | backend | 待实现 |
|---|---|---|
| `radar` | `conti` | ProtocolData + canbus |
| `microphone` | `respeaker` | PortAudio |
| `camera` + smartereye | `smartereye` | 厂商 SDK |
| `rslidar`/`lslidar`/… | stub | 参照 Velodyne/Livox 实现 |

单元测试：`test_skeleton_modules`、`test_canbus_skeleton`。

## GNSS 语句解析

```cpp
auto p = autodriver::gps::GnssParserRegistry::Instance().CreateParser("nmea");
p->Consume(bytes, n);
```

别名：`nmea0183`。

## 诊断

Attach/Detach 结果经 `SampleSink::HandleDiagnostic` → Publisher → `/diagnostics`（可通过 `SetDiagnosticsChannel` 修改话题）。

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
