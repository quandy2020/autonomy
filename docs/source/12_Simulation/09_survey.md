# 9. 仿真技术综述

在 AutoSim、Gazebo、纯 Bridge 之间做选型，而不是比「仿真器打分」。

按 ** fidelity 与依赖** 对照本仓库实际接法；公式见 [§3](03_math.md)。

**仿真器对照（怎么选）**

| 方式 | 物理 | 传感器 | 依赖 | 适合 |
|------|------|--------|------|------|
| launch + Bridge | 无或外部 | 注入 / 录包 | Autolink | CI、栈回归 |
| AutoSim | 后端相关 | IMU、点云、相机 | 脚本 + 可选 GPU | 定位联调 |
| Gazebo + autonomy_ros | ODE/Bullet | 激光、相机丰富 | ROS 2 | 全栈传感器 |
| Stage 配置 | 2D 简单 | 激光（需外部客户端） | 轻 | 规划原型 |

### fidelity 阶梯

```text
运动学限幅 / 注入 odom → Stage 2D 资源 → Gazebo 3D → Isaac / Habitat
```

| 级别 | 代表 | 在本仓库 |
|------|------|----------|
| 无物理注入 | Bridge、mock 时间 | 多进程联调常用 |
| 2D | Stage world 文件 | `map/conf/simulation/`，无内置 Player |
| 3D 物理 | Gazebo | autonomy_ros 外部包 |
| 具身 / RL | Isaac、Habitat | AutoSim 脚本、Docker 可选 |

### 时钟

| 模式 | 驱动 |
|------|------|
| 墙钟 | 系统时间 |
| 仿真 | Gazebo `/clock` 或 `use_sim_time` |
| 单测 / 回放 | Autolink `EnableSimulationMode()` |

### 能力边界（务实）

**已有：** 差速运动学文档、`vehicle` 限幅、多进程 launch、Gazebo/AutoSim 文档与脚本、costmap 吃 `/scan`。

**未有：** 统一 SimulationServer、内置动态障碍、进程内 `nav_test`、Stage/pedsim 的 C++ 驱动、传感器噪声模型产品化。

### 常见坑

| 现象 | 先查 |
|------|------|
| 定位无 `/points` | AutoSim 是否起、yaml 话题名 |
| Gazebo 不动 | `/cmd_vel`、bridge |
| 找 Lightning 定位后端 | 已移除；用 Atlas/Argus |

### 延伸阅读

[§7 Gazebo](07_gazebo_ros.md) · [§6 AutoSim](06_autosim.md) · [§8 Vehicle](08_vehicle_stage.md)
