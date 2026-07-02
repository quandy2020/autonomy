(simulation-overview)=
# 1. 模块概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 仿真层级 | 运动学仿真、物理仿真、传感器仿真、通信仿真 |
| 输入 | `cmd_vel`、仿真世界配置、初始位姿 |
| 输出 | `odom`、TF、`/scan`、相机图像（视仿真后端） |
| 用途 | 无硬件验证 Map → Planning → Control → Navigator 全栈 |
| 对标 | Gazebo、Stage、Webots、Isaac Sim |

### 1.2 核心能力

| 能力 | 状态 | 说明 |
|------|------|------|
| `autonomy_nav_test` 进程内仿真 | ✅ 已实现 | 差速模型积分 odom + TF，跑通完整导航栈 |
| `vehicle::Vehicle` 抽象 | ✅ 部分 | 接口就绪，`ApplyCommand` 待接硬件/仿真 |
| `KinematicsControl` 限幅 | ✅ 已实现 | 速度/加速度约束 |
| Gazebo 集成 | ✅ 外部包 | `autonomy_gazebo` + `autonomy_ros` |
| Stage 配置占位 | ⏳ 配置 | `config/simulation/simulation.lua` + `cave.world` |
| Autolink 仿真模式 | ✅ 已实现 | `MODE_SIMULATION` mock 时间 |
| 独立 `autonomy/simulation/` 模块 | ❌ 未实现 | 无专用源码目录 |

> **当前阶段**：仿真能力**分散**于 `system/tools/nav_test`、外部 ROS 包与 `vehicle` 抽象；无统一 SimulationServer。

### 1.3 相关目录结构

```
# 进程内仿真
autonomy/system/tools/
├── nav_test.cpp              # 离线导航 + 差速仿真
└── README.md

# 车辆抽象（仿真扩展点）
autonomy/vehicle/
├── vehicle.hpp               # VehicleInterface 默认实现
├── vehicle_server.hpp
├── motion/kinematics_control.*
└── common/vehicle_inteface.hpp

# 配置占位
config/simulation/
├── simulation.lua            # Stage / Pedsim 配置
├── world/cave.world          # Stage 2D 世界
└── maps/cave.png

# 外部（不在本仓库）
autonomy_ros / autonomy_gazebo  # ROS2 + Gazebo
```

### 1.4 仿真方式对比

| 方式 | 依赖 | 传感器 | 适用场景 |
|------|------|--------|----------|
| **nav_test** | 无 ROS | 无（静态地图） | CI、算法快速验证 |
| **Gazebo** | ROS2、Gazebo | 激光、相机、odom | 完整物理+传感器 |
| **Stage** | Player/Stage | 2D 激光、相机（规划） | 轻量 2D 仿真 |
| **Autolink MODE_SIMULATION** | Autolink | 无 | 回放、单元测试 |

### 1.5 导航栈数据流（nav_test）

```
autonomy_nav_test
  ├─ cmd_vel ← ControllerServer
  ├─ IntegrateDiffDrive → odom + TF
  └─ NavigateToPose → Map / Planner / Controller / BT
```

### 1.6 相关模块

| 模块 | 关系 |
|------|------|
| `autonomy/system` | `Autonomy::GetLastControlCommand()` 供仿真读取 |
| `autonomy/control` | 输出 `cmd_vel`，接收 `UpdateOdometry` |
| `autonomy/transform` | `Buffer::setTransform` 发布 TF |
| `autonomy/bridge` | ROS 侧桥接 Gazebo topic |
| `autolink` | 仿真时钟与运行模式 |
