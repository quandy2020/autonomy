(simulation-usage)=
# 4. 使用指南

### 4.1 配置

| 项 | 说明 |
|----|------|
| 多进程 | `autolink_launch`；conf：`CreateOptions("autonomy.pb.txt")`；见 [04 Running](../04_Running/03_autonomy_process.md) |
| Stage 占位 | `map/conf/simulation/`；无 SimulationServer 读取 |
| Gazebo | `use_sim_time`/`use_gazebo`/`world_type`/`x_pose`/`y_pose` |

### 4.2 发令

| 方式 | 说明 |
|------|------|
| Bridge | gRPC / 外部下发目标 |
| Autolink Action/Service | 直连 task |
| Gazebo + autonomy_ros | 注入 odom/scan/TF |

进程内 `autonomy_nav_test` / `CreateAutonomy` 已移除。

### 4.3 Vehicle API

```cpp
auto vehicle = std::make_shared<vehicle::Vehicle>("sim_robot");
vehicle->Initialize(model);
vehicle->ApplyCommand(cmd); // 当前仅缓存
vehicle->GetVehicleInfo(&info);
KinematicsControl::ApplyLimits(cmd);
```

### 4.4 Autolink 仿真模式

```cpp
autolink::common::GlobalData::Instance()->EnableSimulationMode();
```

Timer 用 mock 时间；适用于 bag 回放与单测。

### 4.5 构建启动与排错

```bash
cmake -G Ninja -B build && ninja -C build
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
autolink_launch autonomy.launch
```

| 现象 | 处理 |
|------|------|
| 进程未起 | 查 `AUTOLINK_LAUNCH_PATH` / PATH |
| BT 失败 | `AUTONOMY_BT_PLUGIN_PATH` |
| 无激光 / TF 错 | bridge、`/scan`、frame 约定 |

### 4.6 参见
