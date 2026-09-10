# Autodriver

统一传感器 HAL：YAML → 采集 →（可选对齐）→ Autolink。版本见 [`version.json`](version.json)。

**Module 按模态固定；Driver 按厂商 Registry 插拔。** 详设见 [`docs/`](docs/source/index.md)。

## 能力

| 模态 | backend | 状态 |
|---|---|---|
| 相机/点云/板载 IMU | `realsense` `orbbec` | 真；相机可折叠 `streams` |
| 2D 激光 | `rplidar` | 真 |
| 3D 激光 | `velodyne` `hesai` `livox` | 真 |
| IMU/GPS | `serial` `can` | 真 |
| Radar/Mic/SmarterEye | stub | Create→nullptr |

配置：[`config/autodriver_hardware.yaml`](config/autodriver_hardware.yaml)。

## 构建与运行

```bash
# autonomy 仓库根
cmake -S . -B build -DBUILD_AUTODRIVER=ON
cmake --build build -j"$(nproc)" --target autodriver autodriver_main

export AUTODRIVER_PATH=$PWD/src/autonomy/autodriver   # 含 config/
export LD_LIBRARY_PATH=$PWD/build/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/bin:$PATH
autodriver
# 或 autolink launch start autodriver.launch
```

| CMake | 默认 | 依赖 |
|---|---|---|
| `AUTODRIVER_WITH_REALSENSE` | ON | librealsense2 |
| `AUTODRIVER_WITH_ORBBEC` | ON | OrbbecSDK |
| `AUTODRIVER_WITH_RPLIDAR` | ON | rplidar_sdk（`scripts/install_rplidar_sdk.sh`） |
| `AUTODRIVER_WITH_LIVOX` | ON | Livox-SDK/SDK2（`install_livox_sdk*.sh`） |

其它脚本：`create_udev_rules.sh`（`/dev/rplidar`）。

## 目录

```
autodriver/          # 库源码
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
```

RealSense 折叠见主配置 `camera.realsense_d455`。

## 许可证

Apache-2.0（源码头）。厂商 SDK 各从其许可证。
