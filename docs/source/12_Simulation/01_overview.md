(simulation-overview)=
# 1. 模块概览

| 维度 | 说明 |
|------|------|
| 输入 | `cmd_vel`、世界配置、初始位姿 |
| 输出 | `odom`、TF、`/scan`、图像（视后端） |
| 用途 | 无硬件验证 Map → Planning → Control → Task |
| 类似 | Gazebo、Stage、Webots、Isaac Sim |

### 1.2 现状

| 能力 | 状态 | 说明 |
|------|------|------|
| 多进程 + 外部仿真 | — | `autolink_launch` + Gazebo/Bridge |
| `vehicle::Vehicle` | — | 接口就绪，`ApplyCommand` 待接 |
| `KinematicsControl` | — | 速度/加速度限幅 |
| Gazebo | 外部 | `autonomy_gazebo` + `autonomy_ros` |
| Stage 配置 | — | `map/conf/simulation/` |
| Autolink `MODE_SIMULATION` | — | mock 时间 |
| `autonomy/simulation/` | — | 无专用目录 |
| `autonomy_nav_test` | — | 已移除 |

### 1.3 相关路径

```
autonomy/vehicle/ # VehicleInterface, KinematicsControl
autonomy/map/conf/simulation/ # cave.world, cave.png
autonomy/system/launch/autonomy.launch
# 外部：autonomy_ros / autonomy_gazebo
```

### 1.4 方式对比

| 方式 | 依赖 | 适用 |
|------|------|------|
| 多进程 + Bridge | 无 ROS（可） | 栈联调、发令 |
| Gazebo | ROS2 + Gazebo | 物理 + 传感器 |
| Stage | Player/Stage | 轻量 2D（规划） |
| Autolink 仿真模式 | Autolink | 回放、单测 |

### 1.5 数据流

```
外部仿真/Bridge → odom/TF/scan → planning/control/task
cmd_vel ← control；目标 ← Bridge/Action → task
```

### 1.6 参见
