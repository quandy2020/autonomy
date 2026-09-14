(simulation-usage)=
# 4. 使用指南

### 4.1 配置

#### 4.1.1 多进程栈

端到端不经进程内聚合：用 `autolink_launch` 拉起 planning / control / task 等，再由 Bridge 或 Action 发令。见 [04 Running · 多进程栈](../04_Running/03_autonomy_process.md)。

共享 conf：`system::CreateOptions("autonomy.pb.txt")` 加载 `AutonomyOptions`；各进程只取子字段。

#### 4.1.2 simulation 占位（Stage）

历史 `simulation.lua` 占位已迁到地图/仿真资源目录（如 `autonomy/map/conf/simulation/`）。当前无统一 C++ SimulationServer 读取该 conf。

#### 4.1.3 Gazebo Launch 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `use_sim_time` | true | 仿真时钟 |
| `use_gazebo` | false | 启用 Gazebo |
| `world_type` | house | house / world |
| `x_pose`, `y_pose` | -2.0, -0.5 | 初始位姿 |

### 4.2 发令与闭环

| 方式 | 说明 |
|------|------|
| Bridge | gRPC / 外部接口下发目标 |
| autolink Action / Service | 直连 task 等进程 |
| Gazebo + autonomy_ros | 外部仿真注入 odom / scan / TF |

进程内 `autonomy_nav_test` / `CreateAutonomy` **已移除**。

### 4.3 vehicle 模块 API

```cpp
#include "autonomy/vehicle/vehicle.hpp"

auto vehicle = std::make_shared<vehicle::Vehicle>("sim_robot");
vehicle->Initialize(model);
vehicle->ApplyCommand(cmd);  // 当前仅缓存，待接仿真后端
vehicle->GetVehicleInfo(&info);
```

`KinematicsControl::ApplyLimits(cmd)` 对指令限幅。

### 4.4 Autolink 仿真模式

```cpp
autolink::common::GlobalData::Instance()->EnableSimulationMode();
```

影响：Timer 不启动真实轮询、使用 mock 时间。适用于 bag 回放与单元测试。

### 4.5 构建与启动

```bash
cmake -G Ninja -B build && ninja -C build
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
autolink_launch autonomy.launch
```

### 4.6 故障排查

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 进程未起来 | PATH / launch 路径错误 | 检查 `AUTOLINK_LAUNCH_PATH` |
| BT 失败 | 插件未找到 | 设置 `AUTONOMY_BT_PLUGIN_PATH` |
| 规划失败 | 目标在障碍内 | 调整起终点与地图 |
| Gazebo 无激光 | bridge 未连接 | 确认 `autonomy_ros` 与 `/scan` topic |
| TF 错误 | frame 不一致 | 核对 `global_frame` / `base_frame` |

### 4.7 与地图配置配合

地图资产见 `autonomy/map/conf/`。起终点需在自由空间内；frame 约定与共享 conf / 模块 conf 一致。
