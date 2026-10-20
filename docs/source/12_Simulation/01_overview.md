(simulation-overview)=
# 1. 模块概览

选 AutoSim、Gazebo 还是纯 Bridge 注入，先看清仓库里仿真能力散在哪。

用软件代替真机，发出 **odom / TF / scan（及可选图像）**，让 Map → Planning → Control → Task 在无硬件时运行；控制仍发 `cmd_vel`。

| 方式 | 依赖 | 适合 |
|------|------|------|
| launch + Bridge | Autolink | 栈联调、CI |
| AutoSim | 脚本 / 可选 GPU | 定位（Atlas/Argus） |
| Gazebo | ROS 2 | 物理 + 传感器 |
| Stage 资源 only | 外部 Player | 仅当你自有客户端 |

```text
仿真 / Bridge → odom · TF · scan → planning / control / task
cmd_vel ← control；目标 ← Bridge / Action
```

话题名、外参与 **localization conf** 一致；Isaac 环境系统 Python 用 `/usr/bin/python3`。`config/lightning.yaml` 是 AutoSim **场景名**，不是定位后端。

### 仓库里有什么

| 路径 | 作用 |
|------|------|
| [AutoSim](06_autosim.md) | Autolink 仿真桥（fake / Habitat / Isaac） |
| [Gazebo + autonomy_ros](07_gazebo_ros.md) | 外部 ROS 2 包 |
| `vehicle/` | `VehicleInterface`、`KinematicsControl` 限幅 |
| `map/conf/simulation/` | Stage 风格 world（如 cave.world） |
| Autolink 仿真时间 | 单测、mock |

**没有** `autonomy/simulation/` 统一目录；**没有** `SimulationServer`；`autonomy_nav_test` 已删。

### 常见坑

| 现象 | 说明 |
|------|------|
| cave.world 双击无反应 | 需 Stage/Gazebo 等外部程序 |
| AUTONOMY_SIMULATION Lua | 无 C++ 读取 |

### 延伸阅读

[00 指南](00_guide.md)
