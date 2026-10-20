# 0. Simulation 指南

## 背景

仿真里定位无数据、和真机 conf 对不上——仿真只替换 **驱动层**，栈仍是多进程；若话题名、外参与 localization yaml 不一致，后面和真机一样会挂。真机驱动在 `autodriver`，不在本章。

## 思路

实用目标是 **发出和真机同名的 `/imu`、`/points`、图像、TF**，让 localization / navigation 无感。

| 路径 | 何时用 |
|------|--------|
| [06 AutoSim](06_autosim.md) | Autolink 原生仿真桥、Isaac/Habitat（优先） |
| [07 Gazebo](07_gazebo_ros.md) | 已有 Gazebo / ROS 包 |
| [08 Vehicle Stage](08_vehicle_stage.md) | vehicle 抽象与占位 |

- **`config/lightning.yaml` 是 AutoSim 场景名**，不是 localization 后端；找 Lightning 定位 → 用 [Atlas](../06_Localization/08_atlas/guide.md) / [Argus](../06_Localization/06_argus.md) 或 Cartographer。  
- Isaac 环境里 `python` 可能被 alias — 系统脚本用 `/usr/bin/python3`。  
- 端到端仍是 planning/control 原进程，仿真不替它们。


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 概览](01_overview.md) | 仿真路径总览 |
| [02 快速开始](02_quickstart.md) | 最短起仿真 |
| [03 数学](03_math.md) | 模型 |
| [04 用法](04_usage.md) | 配置与话题 |
| [05 架构](05_architecture.md) | 与栈的接缝 |
| [06 AutoSim](06_autosim.md) | Isaac/Habitat、Autolink 桥 |
| [07 Gazebo](07_gazebo_ros.md) | 外部 Gazebo |
| [08 Vehicle Stage](08_vehicle_stage.md) | 底盘抽象 |
| [09 综述](09_survey.md) | 对照 |

**Autolink 原生** → [06](06_autosim.md)。  
**已有 Gazebo 世界** → [07](07_gazebo_ros.md)。  
**只测底盘接口** → [08](08_vehicle_stage.md)。

### 2. 动手时记住

1. 仿真侧改了 topic，定位 yaml 也要改。  
2. 定位联调对照 [Atlas](../06_Localization/08_atlas/guide.md)、[Argus](../06_Localization/06_argus.md)。

### 3. 常见坑

| 现象 | 先查 |
|------|------|
| 定位无数据 | 仿真是否在发；topic 名；时间戳 |
| Isaac 脚本跑飞 | python 是否 Kit 的 alias |
| 找 Lightning 定位 | 已不存在；换 Atlas/Argus |
