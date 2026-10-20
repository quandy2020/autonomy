(instructions-overview)=
# 1. 概览

第一次进仓库需要弄清：软件怎么分层、业务域怎么切、手册其它章节往哪跳。马上要编和跑，不应在本页停留太久。

Autonomy 是 CMake + C++17 的整机软件：驱动、SLAM、感知、导航、机械臂、任务和机上管理。可以不绑 ROS 2；导航语义大体跟 Navigation2 对齐。

按「平台 → 能力域 → 编排 → 应用」理解目录与文档编号；环境就绪后统一走 `setup_environment.bash` + `autolink_launch`。域裁剪用 `AUTONOMY_BUILD_*`，产品件用 `BUILD_*`。

![Autonomy 系统分层架构](./images/autonomy_architecture.png)

## 几个会反复用到的词

- **源码根**：带顶层 `CMakeLists.txt` 的目录；文档路径都相对它。
- **任务编排**：代码在 `autonomy/task`（文档里有时叫 Navigator）。
- **管理面**：`autonomy/system`（monitor / 安全 / OTA / launch）。
- **消息 / 驱动**： `automsgs/`、`autodriver/`。

## 最短验证命令

```bash
source scripts/setup_environment.bash
autolink_launch autonomy.launch
```

更短安装路径见 [02 快速上手](02_quickstart.md) 与 [Installation](../02_Installation/00_guide.md)。

## 域与目录对照

| 你要动的东西 | 大致目录 |
|--------------|----------|
| 底盘与传感 | `autodriver` · `vehicle` · `sensor` |
| 定位建图 | `localization`（模式：`cartographer` · `atlas` · `argus`） |
| 感知预测 | `perception` · `prediction` |
| 导航 | `map` · `planning` · `control` |
| 机械臂 | `automanip`（含 `bolza/`） |
| BT 任务 | `task` |
| 监控 OTA | `system` |
| 通信与对外 | `autolink` · `automsgs` · `bridge` |

平台按 x86-64 / aarch64，推荐 Ubuntu 22.04。

## 手册怎么跳读

![文档结构](./images/docs.png)

01–02 入门安装，03–04 通信与运行，05–09 框架到控制，10–13 感知到可视化，14–16 消息、Bridge、编排，17–20 Navigator、Tools、FAQ、其它。不必按编号通读；从 [00 指南](00_guide.md) 按目标跳。

本地编文档：

```bash
cd docs && pip install -r requirements.txt && sphinx-build -b html source build
```

在线：https://autonomy.readthedocs.io/en/latest/index.html 。版本 **0.2.0**（`version.json`），C++17、CMake ≥ 3.20、Apache 2.0。变更见 [`CHANGELOG.rst`](https://github.com/quandy2020/autonomy/blob/main/CHANGELOG.rst)。
