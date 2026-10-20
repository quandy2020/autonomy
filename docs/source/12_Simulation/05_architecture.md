# 5. Simulation 架构

理解仿真数据从哪进栈、cmd_vel 从哪出；**不是**找 `SimulationServer` 进程。

描述 **多进程 Autonomy + 外部物理/Bridge** 的常见拓扑。仓库内无 `autonomy/simulation/` 统一模块。

| 能力 | 说明 |
|------|------|
| `autolink_launch autonomy.launch` | 规划/控制/任务等多进程，传感器靠 Bridge 或录包注入 |
| AutoSim | Autolink 仿真桥，见 [§6](06_autosim.md) |
| Gazebo + autonomy_ros | 外部 ROS 包，见 [§7](07_gazebo_ros.md) |
| `vehicle/` + `KinematicsControl` | 限幅与接口；`ApplyCommand` 当前多仅缓存 |
| `map/conf/simulation/` | Stage 风格 world 资源，**无** C++ Stage 客户端读 AUTONOMY_SIMULATION |
| `SimulationServer`（文档旧名） | **未实现** |
| `autonomy_nav_test` | 已移除 |

| 场景 | 路径 |
|------|------|
| CI / 栈联调 | launch + Bridge |
| 要激光相机模型 | Gazebo 或 AutoSim |
| 只测限幅 | vehicle + KinematicsControl |

### 分层

| 层 | 组件 |
|----|------|
| 应用 | Task / Planning / Control |
| 进程 | `autonomy.launch` 拆分的 bin |
| 外部 | Gazebo、AutoSim、Bridge（odom、scan、TF） |
| 桥接 | `autonomy.bridge`、autonomy_ros、`VehicleInterface` |

### 数据流

```text
ControllerServer ── cmd_vel ──→ Bridge / ROS ──→ Gazebo 或 AutoSim
        ↑                              │
        └──── /odom, /scan, TF ←───────┘
```

无物理时：Bridge 注入 odom/TF；任务目标经 Action / gRPC。

### 常见坑

| 误解 | 事实 |
|------|------|
| cave.world 会自动跑 | 只是地图资源，需外部仿真器 |
| Lua AUTONOMY_SIMULATION 已读 | 无 SimulationServer 加载 |
| 仿真=单进程 GUI | 仍是多 bin + 外部源 |

### 延伸阅读

[§9 综述](09_survey.md) · [§4 用法](04_usage.md)
