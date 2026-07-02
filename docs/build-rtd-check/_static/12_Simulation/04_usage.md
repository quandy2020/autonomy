(simulation-usage)=
# 4. 使用指南

### 4.1 配置

#### 4.1.1 nav_test 命令行

| 参数 | 默认 | 说明 |
|------|------|------|
| `configuration_directory` | （必填） | 配置根目录 |
| `start_x/y/yaw` | 1, 1, 0 | 初始位姿 [m, rad] |
| `goal_x/y/yaw` | 5, 5, 0 | 目标位姿 |
| `use_bt` | true | 是否走行为树 |
| `timeout_sec` | 120 | 导航超时 [s] |
| `sim_dt` | 0.1 | 仿真步长 [s] |
| `global_frame` | map | 与 `AUTONOMY_COMMON.global_frame` 一致 |
| `base_frame` | base_link | 机器人基座 frame |

#### 4.1.2 simulation.lua（Stage 占位）

```lua
-- config/simulation/simulation.lua
AUTONOMY_SIMULATION = {
    stage = {
        world_file = "configuration_files/simulation/world/willow-erratic.world",
        map_file = "configuration_files/simulation/map/willow-erratic.yaml",
    },
    pedsim = { },
}
```

> **注意**：`AUTONOMY_SIMULATION` 未被 `config/autonomy.lua` include，无 C++ 读取逻辑。

#### 4.1.3 Gazebo Launch 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `use_sim_time` | true | 仿真时钟 |
| `use_gazebo` | false | 启用 Gazebo |
| `world_type` | house | house / world |
| `x_pose`, `y_pose` | -2.0, -0.5 | 初始位姿 |

### 4.2 autonomy_nav_test API

入口：`autonomy::system::tools::RunNavTest(argc, argv)`

内部流程：

1. `CreateOptions` + `CreateAutonomy`
2. `Start()` → Map / Planner / Controller / Transform
3. `Configure()` → BT 导航引擎
4. 注入初始 odom + TF
5. 后台线程：`GetLastControlCommand` → `IntegrateDiffDrive` → `UpdateOdometry` + TF
6. `NavigateToPose` 直至成功/失败/超时

### 4.3 system::Autonomy 仿真相关 API

| API | 说明 |
|-----|------|
| `GetLastControlCommand()` | 返回最新 `cmd_vel` 供仿真器积分 |
| `TickControl()` | 单步控制（直驱模式） |
| `NavigateToPose(goal)` | 发起导航任务 |
| `NavigateDirectToPose(goal)` | 直驱规划+跟随 |

### 4.4 vehicle 模块 API

```cpp
#include "autonomy/vehicle/vehicle.hpp"

auto vehicle = std::make_shared<vehicle::Vehicle>("sim_robot");
vehicle->Initialize(model);
vehicle->ApplyCommand(cmd);  // 当前仅缓存，待接仿真后端
vehicle->GetVehicleInfo(&info);
```

`KinematicsControl::ApplyLimits(cmd)` 对指令限幅。

### 4.5 Autolink 仿真模式

```cpp
autolink::common::GlobalData::Instance()->EnableSimulationMode();
```

影响：Timer 不启动真实轮询、使用 mock 时间。适用于 bag 回放与单元测试。

### 4.6 构建选项

```bash
cmake .. -DBUILD_TOOLS=ON -DBUILD_TASKS=ON
ninja autonomy_nav_test
```

### 4.7 故障排查

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `Autonomy not ready` | BT 配置失败 | 检查 `AUTONOMY_BT_PLUGIN_PATH` |
| 规划失败 | 目标在障碍内 | 调整起终点；用 `autonomy_planning_test` 可视化 |
| 超时 | 控制器未收敛 | 增大 `timeout_sec` |
| Gazebo 无激光 | bridge 未连接 | 确认 `autonomy_ros` 与 `/scan` topic |
| TF 错误 | frame 不一致 | 核对 `global_frame` / `base_frame` |

### 4.8 与地图配置配合

默认地图 `config/data/map.pgm`，起终点需在自由空间内。`config/common.lua` 中：

```lua
AUTONOMY_COMMON = {
    global_frame = "map",
    robot_base_frame = "base_link",
}
```
