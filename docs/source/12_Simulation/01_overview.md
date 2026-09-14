(simulation-overview)=
# 1. 模块概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 仿真层级 | 运动学仿真、物理仿真、传感器仿真、通信仿真 |
| 输入 | `cmd_vel`、仿真世界配置、初始位姿 |
| 输出 | `odom`、TF、`/scan`、相机图像（视仿真后端） |
| 用途 | 无硬件验证 Map → Planning → Control → Task 全栈 |
| 对标 | Gazebo、Stage、Webots、Isaac Sim |

### 1.2 核心能力

| 能力 | 状态 | 说明 |
|------|------|------|
| 多进程栈 + 外部仿真 | ✅ 推荐 | `autolink_launch autonomy.launch` + Gazebo/Bridge |
| `vehicle::Vehicle` 抽象 | ✅ 部分 | 接口就绪，`ApplyCommand` 待接硬件/仿真 |
| `KinematicsControl` 限幅 | ✅ 已实现 | 速度/加速度约束 |
| Gazebo 集成 | ✅ 外部包 | `autonomy_gazebo` + `autonomy_ros` |
| Stage 配置占位 | ⏳ 配置 | `autonomy/map/conf/simulation/` 等 |
| Autolink 仿真模式 | ✅ 已实现 | `MODE_SIMULATION` mock 时间 |
| 独立 `autonomy/simulation/` 模块 | ❌ 未实现 | 无专用源码目录 |
| 进程内 `autonomy_nav_test` | ❌ 已移除 | 不再提供单进程差速积分入口 |

> **当前阶段**：仿真能力**分散**于外部 ROS/Gazebo 包与 `vehicle` 抽象；无统一 SimulationServer。端到端用多进程栈，不经进程内聚合。

### 1.3 相关目录结构

```
# 车辆抽象（仿真扩展点）
autonomy/vehicle/
├── vehicle.hpp               # VehicleInterface 默认实现
├── vehicle_server.hpp
├── motion/kinematics_control.*
└── common/vehicle_inteface.hpp

# 配置占位（地图 / Stage 世界）
autonomy/map/conf/simulation/
├── world/cave.world
└── maps/cave.png

# 多进程入口
autonomy/system/launch/autonomy.launch

# 外部（不在本仓库）
autonomy_ros / autonomy_gazebo  # ROS2 + Gazebo
```

### 1.4 仿真方式对比

| 方式 | 依赖 | 传感器 | 适用场景 |
|------|------|--------|----------|
| **多进程 + Bridge** | 无 ROS（可） | 由外部注入 | 栈联调、发令验证 |
| **Gazebo** | ROS2、Gazebo | 激光、相机、odom | 完整物理+传感器 |
| **Stage** | Player/Stage | 2D 激光、相机（规划） | 轻量 2D 仿真 |
| **Autolink MODE_SIMULATION** | Autolink | 无 | 回放、单元测试 |

### 1.5 导航栈数据流（多进程）

```
外部仿真 / Bridge
  ├─ odom / TF / scan → planning / control / task
  ├─ cmd_vel ← autonomy.control
  └─ 目标 ← Bridge / Action Client → autonomy.task
```

### 1.6 相关模块

| 模块 | 关系 |
|------|------|
| `autonomy/control` | 输出 `cmd_vel`，接收里程计 |
| `autonomy/transform` | TF 缓冲与静态外参 |
| `autonomy/bridge` | ROS / 外部仿真 topic 桥接 |
| `autonomy/task` | BT / Action 发令入口 |
| `autolink` | 仿真时钟与运行模式 |
