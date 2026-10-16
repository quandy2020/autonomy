# autosim

多后端**传感器–执行器桥**：经 Autolink 发布传感、订阅 `/cmd_vel`，并对接 TF / 仿真时钟 / 可选真值地图。  
**不实现** MapService / NavigationService / LocalizationService RPC。

![Isaac Sim + Autoviz](images/isaac.gif)

> Isaac（酷家乐 InteriorAgent）与 Autoviz：RGB-D、`/map`、`/overall/map`、PhysX。完整文档见 [`docs/`](docs/)。

## 文档

| § | 文档 | 内容 |
|---|------|------|
| 00 | [指南](docs/00_guide.md) | 边界、术语、阅读路径 |
| 01–02 | [概述](docs/01_overview.md) · [架构](docs/02_architecture.md) | 能力与数据流 |
| 03–04 | [快速开始](docs/03_quickstart.md) · [后端](docs/04_backends.md) | 安装启动与三后端 |
| 05–06 | [配置](docs/05_configuration.md) · [话题](docs/06_topics.md) | YAML 与 Autolink 通道 |
| 07–08 | [Isaac](docs/07_isaac.md) · [地图与可视化](docs/08_map_and_viz.md) | USD / PhysX / Autoviz |
| 09–10 | [测试](docs/09_testing.md) · [排错](docs/10_troubleshooting.md) | pytest 与常见问题 |
| 11 | [数据采集](docs/11_dataset.md) | LeRobot v2.1 模仿学习落盘 |

索引：[`docs/README.md`](docs/README.md)。

## 包结构

| 层 | 路径 | 职责 |
|----|------|------|
| 编排 | `autosim/runtime/` | Runner 控制环、SensorWorker |
| 契约 | `autosim/backend/` | WorldBackend、registry、capabilities |
| 引擎 | `autosim/backends/{habitat,fake,isaac}/` | 互不耦合的后端实现 |
| 几何 | `autosim/scenario/` | 静态图 + 动态障碍/行人 |
| 传感 / I/O | `autosim/sensing/` · `autosim/io/` | 采样噪声、Bridge、Teleop |
| 地图 / 配置 | `autosim/map/` · `autosim/config/` | 真值占据、YAML 校验 |
| 数据集 | `autosim/dataset/` | LeRobot 采集（模仿学习基础） |
| 策略 / 机器人 | `autosim/policy/` · `autosim/robot/` | ONNX Twist、URDF、差速植物 |

详见 [架构](docs/02_architecture.md) · [数据集](docs/11_dataset.md)。

## 后端一览

| `habitat.backend` | 配置 | 说明 |
|-------------------|------|------|
| `habitat` | `config/default.yaml` | Habitat-Sim + GLB / MP3D |
| `fake` | `config/fake.yaml` | 无引擎：差速 + RGBD raycast + scenario |
| `isaac` | `config/isaac.yaml` | Isaac Sim + USD；PhysX、相机、真值地图、动态体 |

## 快速命令

```bash
cd /workspace/autonomy

# Fake
CONFIG=src/autonomy/autosim/config/fake.yaml src/autonomy/autosim/scripts/run.sh sim

# Isaac（需 Isaac Lab Python）
ISAACLAB_PATH=/workspace/isaaclab \
CONFIG=src/autonomy/autosim/config/isaac.yaml \
  src/autonomy/autosim/scripts/run.sh sim

# 遥控（另终端，需 TTY 焦点）
src/autonomy/autosim/scripts/run.sh teleop
```

## 资源

| 路径 | 内容 |
|------|------|
| [`docs/`](docs/) | 00–10 维护文档 |
| [`images/`](images/) | 演示动图 |
| [`usd/`](usd/) | Isaac USD（大场景见 `usd/.gitignore`） |
| [`config/`](config/) | YAML |
| [`robot/`](robot/) | 机器人 URDF + meshes |
| [`tests/`](tests/) | pytest |

## 安装（摘要）

```bash
cd src/autonomy/autosim
pip install -e ".[dev]"            # 测试
pip install -e ".[habitat,dev]"    # Habitat
pip install -e ".[isaac,dev]"      # Isaac extras（仍需 Isaac 环境）
```

`colcon build --packages-select autonomy` 时若 `BUILD_AUTOSIM=ON`（默认）会 `pip install -e autosim`。

细节、配置字段、话题表与排错见 [`docs/`](docs/)。
