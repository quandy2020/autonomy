# Autodriver

统一硬件 HAL：传感 + 本体（chassis）。YAML → 采集/执行 → Autolink。  
版本：CMake 配置时用 git 刷新 [`version.json`](version.json)（`full_version` / `git_*`）；CLI：`autodriver -V`。打标签 `autodriver-vX.Y.Z` 可同步 semver。

**Module 按模态固定；Driver 按厂商 Registry 插拔。** 详设见 [`docs/`](docs/source/index.md)。

| 域 | 路径 | 说明 |
|---|---|---|
| 传感 | `camera/` `lidar/` … | `SensorDriver`，单向采样 |
| 本体 | [`chassis/`](chassis/README.md) | Capability + Mode + SafetyGate；`RobotState` / `RobotEvent` + Twist；**不依赖** `autonomy/vehicle` |
| 手柄 | `autodriver/joy/` | `/dev/input/js*` → `/joy` + `/cmd_vel`（按住使能） |

## 传感器支持范围

下列为当前代码与配置中的**支持范围**（Registry 已注册、可走采集路径或已有 params）。  
**实际可用性取决于具体硬件型号**，以及本机是否安装对应 SDK、CMake `AUTODRIVER_WITH_*` / `AUTODRIVER_HAVE_*`、权限与网段；未找到 SDK 时 Create 常返回 `nullptr`，不阻碍编译链接。厂商细项与排障见 [`docs/source/sensor/`](docs/source/sensor/index.md)。

| 模态 | YAML | backend | 覆盖型号 / 说明 | 驱动状态 |
|---|---|---|---|---|
| 相机 / 点云 / 板载 IMU | `camera`（可折叠 `streams` / `point_clouds` / `imu`） | `realsense` | **Intel RealSense D455** 为主；同机多流共享 device hub；可用 `model` / `serial` / `index` 选设备 | 已实现（需 librealsense2） |
| | | `orbbec` | **Orbbec Gemini 330** 为主；同机多流共享 hub | 已实现（需 OrbbecSDK） |
| | | `smartereye` | 占位 | stub（Create 返回 `nullptr`） |
| 2D 激光 | `lidar_2d` | `rplidar`（别名 `slamtec`） | **Slamtec A1 / A2（含 A2M8）/ A3**；A2M7/M12、A3 常用波特率 256000；`params_file: lidar/slamtec/{a1,a2,a3}.yaml` | 已实现（需 rplidar_sdk） |
| 3D 激光 | `lidar_3d` | `velodyne`（别名 `udp`） | **Velodyne VLP-16** 为主；UDP 自研栈，校准单位 rad | 已实现 |
| | | `hesai`（别名 `pandar`） | **Hesai PandarXT / XT32**；包格式未覆盖 XT32M2X 等；校准单位为度 | 已实现 |
| | | `livox` | **SDK2**：HAP、Mid-360、Mid360s、Avia2；**SDK1**：Mid-40/70、Horizon、Avia、Tele；按 `model`/`sdk` 选型 | 已实现（需对应 Livox SDK） |
| | | `rslidar` / `lslidar` / `seyond` / `vanjee` 等 | 占位注册 | stub |
| IMU | `imu` | `serial` | 串口协议（如 WitMotion）；`port` / `baudrate` | 已实现 |
| | | `can` | SocketCAN 分帧 | 已实现 |
| | | `realsense` | 板载 IMU（随 RealSense 模组） | 已实现（需 librealsense2） |
| GPS | `gps` | `serial` | NMEA（`GnssParserRegistry`：`nmea` / `nmea0183`） | 已实现 |
| | | `can` | CAN 帧 | 已实现 |
| Radar | `radar` | `conti`（别名 `continental`） | 占位 | stub |
| 麦克风 | `microphone` | `respeaker` | 占位（Image 承载 PCM） | stub |
| 测距 | `range` | — | 仅 Attach，无采集驱动 | 仅 Attach |
| 底盘 / 本体 | `chassis` | `stub`（别名可 `sim`） | 差分积分，无硬件联调 | 可联调；厂商经 `REGISTER_CHASSIS_BACKEND` 扩展 |
| 手柄遥操 | `joy` | — | **默认索尼 DualSense（PS5）**：左摇杆差速 + **按住 L1** 使能 → `/cmd_vel`；并发布 `/joy`。预设 `config/joy/dualsense.yaml` | 已实现（Linux；无设备时告警并空转） |

配置入口：[`config/autodriver_hardware.yaml`](config/autodriver_hardware.yaml)；厂商 params 在 `config/<模态>/<vendor>/`。

## 构建与运行

```bash
# autonomy 仓库根（colcon / 嵌套构建产物在 build/autonomy/）
cmake --build build/autonomy -j"$(nproc)" --target autodriver autodriver_main

export AUTODRIVER_PATH=$PWD/src/autonomy/autodriver
export LD_LIBRARY_PATH=$PWD/build/autonomy/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/autonomy/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$AUTODRIVER_PATH/launch

autodriver
# 或（二选一，勿同时开两个实例——会抢 RealSense）
autolink launch start autodriver.launch
```

> 若仍指向旧的 `build/lib/libautodriver.so`，会出现  
> `undefined symbol: …ChassisManager::Start…`——请改用上面的 `build/autonomy/lib`，或重新编译后覆盖旧库。
>
> RealSense `module Start failed` 且错误含 `Device or resource busy`：先 `pkill -f autodriver` 再启一次。

| CMake | 默认 | 依赖 |
|---|---|---|
| `AUTODRIVER_WITH_REALSENSE` | ON | librealsense2 |
| `AUTODRIVER_WITH_ORBBEC` | ON | OrbbecSDK |
| `AUTODRIVER_WITH_RPLIDAR` | ON | rplidar_sdk（`scripts/install_rplidar_sdk.sh`） |
| `AUTODRIVER_WITH_LIVOX` | ON | Livox-SDK/SDK2（`install_livox_sdk*.sh`） |

其它脚本：`create_udev_rules.sh`（`/dev/rplidar`）。

## 目录

```
autodriver/          # 传感库源码
chassis/             # 本体/底盘 HAL（独立子目录）
config/              # 硬件 YAML + 厂商 params
scripts/             # SDK / udev
launch/ docs/ test/ examples/
main.cpp
```

## 文档

| 页 | 内容 |
|---|---|
| [docs 首页](docs/source/index.md) | 导航 |
| [架构](docs/source/guide/architecture.md) | 分层、Registry |
| [数据流](docs/source/guide/dataflow.md) | 采集路径 |
| [使用](docs/source/guide/usage.md) | 进程/嵌入 |
| [配置](docs/source/guide/configuration.md) | YAML |
| [测试](docs/source/guide/testing.md) | ctest |
| [传感器](docs/source/sensor/index.md) | 厂商 |
| [API](docs/source/api/overview.md) | C++ |
| [FAQ](docs/source/faq.md) | 排障 |

```bash
pip install -r docs/requirements.txt && cd docs && mkdocs serve
```

## 配置例

```yaml
# RPLidar
lidar_2d:
  - {name: front, enable: true, backend: rplidar, port: /dev/ttyUSB0,
     channel: /lidar/front/scan, params_file: lidar/slamtec/a1.yaml}

# Livox Mid-360
lidar_3d:
  - {name: mid360, enable: true, backend: livox,
     channel: /lidar/mid360/points, params_file: lidar/livox/mid360.yaml}

# 手柄遥操（DualSense：按住 L1，左摇杆控制底盘）
joy:
  enable: true
  profile: dualsense
  device: /dev/input/js0
  max_linear: 0.5
  max_angular: 1.0
```

RealSense 折叠见主配置 `camera.realsense_d455`。

## 许可证

Apache-2.0（源码头）。厂商 SDK 各从其许可证。
