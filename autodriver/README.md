# Autodriver

统一硬件 HAL：**传感** + **本体（chassis）** + **手柄遥操**。YAML 配置 → 采集/执行 → Autolink。

- Module 按模态固定；Driver 按厂商 Registry 插拔
- 版本：`autodriver -V`（构建时刷新 [`version.json`](version.json)）
- 详设：[`docs/`](docs/source/index.md) · 本体：[`chassis/README.md`](chassis/README.md)

## 一览

| 域 | 路径 | 做什么 |
|---|---|---|
| 传感 | `autodriver/{camera,lidar,…}/` | `SensorDriver` → Autolink |
| 本体 | [`chassis/`](chassis/README.md) | `/cmd_vel` + mode/tool → `RobotState` / odom / event |
| 手柄 | `autodriver/joy/` | DualSense → `/joy` + `/cmd_vel`（按住 L1） |
| 启动 | [`launch/`](launch/autodriver.launch) · [`dag/`](dag/) | 传感 binary + 底盘 DAG **二选一** |

## 传感（摘要）

| 模态 | backend（已实现） | 备注 |
|---|---|---|
| 相机 / 点云 | `realsense`（D455）、`orbbec`（Gemini 330） | 需厂商 SDK |
| 2D 激光 | `rplidar` | A1/A2/A3；需 rplidar_sdk |
| 3D 激光 | `velodyne`、`hesai`、`livox` | Livox 需 SDK1/2 |
| IMU / GPS | `serial` / `can` | NMEA、WitMotion 等 |
| Radar / Mic | stub | 占位 |

完整表与排障：[`docs/source/sensor/`](docs/source/sensor/index.md)。

## 底盘

| backend | 库 / Component | 配置 | DAG |
|---|---|---|---|
| `stub` | 链入 `libautodriver.so` | YAML `backend: stub` | — |
| `jetauto` | `libautodriver_jetauto.so` · `JetAutoComponent` | [`config/chassis/jetauto.yaml`](config/chassis/jetauto.yaml) | [`dag/chassis_jetauto.dag`](dag/chassis_jetauto.dag) |
| `l1w` | `libautodriver_l1w.so` · `L1wComponent` | [`config/chassis/l1w.yaml`](config/chassis/l1w.yaml) | [`dag/chassis_l1w.dag`](dag/chassis_l1w.dag) |

- **JetAuto**：幻尔 RRC USB；麦轮 `omni` / 差分 `differential`
- **L1-W**：钢镚 ZSL-1W 全 HighLevel（`stand`/`wheel`→move、`walk`→crawl；tools `lie`/`passive`/`attitude`…）；有线默认 `192.168.168.168`；可选 `-DGenisomL1w_ROOT=`

细项：[chassis](chassis/README.md) · [L1-W](chassis/l1w/README.md)

## 运行

```bash
# 仓库根构建
cmake --build build/autonomy -j"$(nproc)" --target autodriver autodriver_main

export AUTODRIVER_PATH=$PWD/src/autonomy/autodriver   # 或 $PWD/autodriver
export LD_LIBRARY_PATH=$PWD/build/autonomy/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/autonomy/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$AUTODRIVER_PATH/launch
export AUTOLINK_DAG_PATH=$AUTODRIVER_PATH/dag
export AUTOLINK_LIB_PATH=$PWD/build/autonomy/lib
```

| 方式 | 命令 | 说明 |
|---|---|---|
| 仅传感进程 | `autodriver` | 读 `config/autodriver_hardware.yaml`（默认 `chassis.enable: false`） |
| Launch | `autolink launch start autodriver.launch` | 传感 + **一个**底盘 DAG |
| 仅底盘 | `mainboard -d $AUTOLINK_DAG_PATH/chassis_l1w.dag` | JetAuto 换 `chassis_jetauto.dag` |

**底盘二选一**：编辑 [`launch/autodriver.launch`](launch/autodriver.launch)，只启用 `chassis_l1w` 或 `chassis_jetauto` 其中一个 `<module>`。

不要同时跑 `autodriver` 与 launch（会抢 RealSense）。旧库路径会导致 `ChassisManager::Start` 符号缺失 → 用 `build/autonomy/lib`。

| CMake | 作用 |
|---|---|
| `AUTODRIVER_WITH_{REALSENSE,ORBBEC,RPLIDAR,LIVOX}` | 传感 SDK（默认 ON；未找到不挡编译） |
| `GenisomL1w_ROOT` | 可选 L1-W HighLevel SDK |

## 目录

```text
autodriver/     传感 + joy
chassis/        stub · jetauto · l1w + Manager
dag/            chassis_jetauto.dag · chassis_l1w.dag
config/         YAML（chassis/ joy/ 厂商 params）
launch/         autodriver.launch
docs/ test/ scripts/ examples/
```

## 文档

| | |
|---|---|
| [快速开始](docs/source/guide/quickstart.md) | 构建 / 环境 / launch |
| [使用](docs/source/guide/usage.md) · [配置](docs/source/guide/configuration.md) | 进程、YAML |
| [后端](docs/source/guide/backends.md) · [架构](docs/source/guide/architecture.md) | Registry、加厂商 |
| [本体](docs/source/guide/chassis.md) | chassis 通道、DAG、JetAuto / L1-W |
| [传感器](docs/source/sensor/index.md) · [FAQ](docs/source/faq.md) | 厂商、排障 |

```bash
cd docs && pip install -r requirements.txt && mkdocs serve
```

## 许可证

Apache-2.0。厂商 SDK 各从其许可证。
