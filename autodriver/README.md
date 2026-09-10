# Autodriver

统一传感器硬件抽象层：YAML 配置 → 内置 `SensorModule` 采集 → 可选时间对齐 →
`bridge::Publisher` 发布到 Autolink。

版本见 [`version.json`](version.json)（当前 0.1.0）。

## 功能

| 模态 | 厂商 / backend | 状态 |
|---|---|---|
| 相机 / 点云 / 板载 IMU | Intel RealSense、Orbbec | 真采集 |
| 2D 激光 | Slamtec RPLidar（`rplidar`） | 真采集 |
| 3D 激光 | Velodyne、Hesai XT32、Livox（SDK1+SDK2） | 真采集 |
| IMU / GPS | 串口 WitMotion、NMEA；CAN | 真采集 |
| Radar / Mic / SmarterEye | Conti / Respeaker / … | stub |

配置入口：[`config/autodriver_hardware.yaml`](config/autodriver_hardware.yaml)。  
相机推荐「一台设备 + `streams` / `point_clouds` / `imu`」折叠写法。

## 快速开始

在 **autonomy 仓库根目录**（含顶层 `CMakeLists.txt`）：

```bash
cmake -S . -B build -DBUILD_AUTODRIVER=ON
cmake --build build -j"$(nproc)" --target autodriver autodriver_main

export AUTODRIVER_PATH=$PWD/src/autonomy/autodriver   # 含 config/
# 若路径不同，指向本包根目录（其下有 config/autodriver_hardware.yaml）
export LD_LIBRARY_PATH=$PWD/build/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/bin:$PATH

autodriver
# 或
autolink launch start autodriver.launch
```

编辑 YAML：将需要的传感器设为 `enable: true`。

### 常用 CMake 选项

| 选项 | 默认 | 说明 |
|---|---|---|
| `AUTODRIVER_WITH_REALSENSE` | ON | librealsense2 |
| `AUTODRIVER_WITH_ORBBEC` | ON | OrbbecSDK |
| `AUTODRIVER_WITH_RPLIDAR` | ON | 系统安装的 rplidar_sdk |
| `AUTODRIVER_WITH_LIVOX` | ON | Livox-SDK / Livox-SDK2 |

厂商 SDK 安装脚本（在本包根目录）：

```bash
./scripts/install_rplidar_sdk.sh
./scripts/install_livox_sdk2.sh
./scripts/install_livox_sdk.sh
./scripts/create_udev_rules.sh          # RPLidar → /dev/rplidar
```

## 目录结构

```
autodriver/
  autodriver/          # 库源码（common / camera / lidar / imu / gps / bridge …）
  config/              # autodriver_hardware.yaml + 厂商 params
  scripts/             # SDK / udev / 校验脚本
  launch/              # autodriver.launch
  docs/                # MkDocs（见下）
  examples/            # autodriver_demo
  test/
  main.cpp             # 进程入口
```

## 文档

| 文档 | 内容 |
|---|---|
| [docs 首页](docs/source/index.md) | 架构与阅读路径 |
| [架构与模块化](docs/source/guide/architecture.md) | 分层、Registry、扩展厂商 |
| [数据流](docs/source/guide/dataflow.md) | 采集到 Autolink 路径 |
| [使用方式](docs/source/guide/usage.md) | 进程 / launch / 嵌入 |
| [测试](docs/source/guide/testing.md) | ctest 与用例 |
| [传感器手册](docs/source/sensor/index.md) | 各厂商说明与用法 |
| [快速开始](docs/source/guide/quickstart.md) | 构建 / 运行 / launch |
| [配置](docs/source/guide/configuration.md) | YAML 字段 |
| [后端](docs/source/guide/backends.md) | backend 与扩展方式 |
| [API](docs/source/api/overview.md) | C++ API |
| [FAQ](docs/source/faq.md) | 路径、权限、stub |

本地预览：

```bash
pip install -r docs/requirements.txt
cd docs && mkdocs serve
# 或：cmake --build build --target docs
```

## 配置片段

**RealSense（折叠）** — 见主配置 `camera.realsense_d455`。

**RPLidar：**

```yaml
lidar_2d:
  - name: front
    enable: true
    backend: rplidar
    port: /dev/ttyUSB0
    channel: /lidar/front/scan
    params_file: lidar/slamtec/a1.yaml
```

**Livox Mid-360：**

```yaml
lidar_3d:
  - name: mid360
    enable: true
    backend: livox
    channel: /lidar/mid360/points
    params_file: lidar/livox/mid360.yaml
```

## 许可证

Apache License 2.0（见各源文件头）。厂商 SDK 遵循各自许可证
（Slamtec / Livox / Intel / Orbbec 等）。
