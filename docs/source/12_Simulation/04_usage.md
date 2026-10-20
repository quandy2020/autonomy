(simulation-usage)=
# 4. 使用指南

配 launch、Vehicle 接口或 Autolink 仿真时间模式。

说明仿真相关 **配置入口、发令方式、vehicle 抽象**，不描述不存在的 SimulationServer。

| 场景 | 做法 |
|------|------|
| 全栈无 Gazebo | launch + 注入 |
| 测限幅 | vehicle + KinematicsControl 单测 |
| 2D world 文件 | 仅资源，需外部 Stage |

### 配置

| 项 | 说明 |
|----|------|
| 多进程 | `autolink_launch`；conf 见 [04 Running · 进程](../04_Running/03_autonomy_process.md) |
| Stage 资源 | `map/conf/simulation/`；**无** Server 读 AUTONOMY_SIMULATION |
| Gazebo | launch 参数 `use_sim_time`、`use_gazebo`、`world_type`、初始位姿 |

### 发令

| 方式 | 说明 |
|------|------|
| Bridge gRPC | 外部目标 |
| Autolink Action/Service | task |
| autonomy_ros | cmd_vel、注入 odom/scan |

进程内 `CreateAutonomy` / `nav_test` 已移除。

### Vehicle API

```cpp
auto vehicle = std::make_shared<vehicle::Vehicle>("sim_robot");
vehicle->Initialize(model);
vehicle->ApplyCommand(cmd); // 当前多仅缓存
KinematicsControl::ApplyLimits(cmd);
```

对接 Gazebo / 真底盘需自写 `VehicleInterface` 实现或走 ROS cmd_vel。

### Autolink 仿真模式

```cpp
autolink::common::GlobalData::Instance()->EnableSimulationMode();
```

Timer 用 mock 时间；适合 bag 回放与单测。

### 常见坑

| 现象 | 处理 |
|------|------|
| 进程未起 | PATH、`AUTOLINK_LAUNCH_PATH` |
| TF 错 | 仿真 frame 与 URDF |
| 以为 Lua stage 会加载 | 无读取代码 |

### 延伸阅读

[§8 Vehicle](08_vehicle_stage.md) · [§5 架构](05_architecture.md)
