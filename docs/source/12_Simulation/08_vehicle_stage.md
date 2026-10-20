# 8. 车辆抽象与 Stage

### 8.1 结构

```
VehicleServer → Vehicle (VehicleInterface)
Initialize / ApplyCommand / GetVehicleInfo
└─ KinematicsControl::ApplyLimits
```

### 8.2 VehicleInterface

文件：`vehicle/common/vehicle_inteface.hpp`。

| 方法 | 说明 |
|------|------|
| `Initialize(model)` | 尺寸、动力学约束 |
| `GetVehicleInfo` | 速度等状态 |
| `ApplyCommand` | 下发指令（当前仅缓存） |

对接目标：Gazebo 差速插件、真实底盘、运动学后端。

### 8.3 KinematicsControl / VehicleModel

| 约束 | 字段 |
|------|------|
| 线/倒车/角速度 | `max_linear_speed`、`max_reverse_speed`、`max_angular_speed` |
| 线/角加速度 | `max_linear_acceleration`、`max_angular_acceleration` |

Proto：`vehicle/proto/vehicle_options.proto`（尺寸、diff/holonomic/ackermann、上限）。公式：[§3.6](03_math.md)。

### 8.4 Stage 配置

```lua
AUTONOMY_SIMULATION = {
    stage = {
        world_file = ".../willow-erratic.world", -- 与仓库 cave.world 路径可能不一致
        map_file = ".../willow-erratic.yaml",
    },
    pedsim = {}, -- 行人占位，未实现
}
```

| cave.world 元素 | 说明 |
|-----------------|------|
| `pioneer2dx_with_laser` | `robot_0` |
| `hokuyolaser` | 270° 激光 |
| `camera_realsens_d435` | RGB-D |
| floorplan cave | 16×16 m（`maps/cave.png`） |

Player/Stage 格式；无本仓库 C++ 客户端。

### 8.5 后端规划

```cpp
// StageSimulator / GazeboSimulator : VehicleInterface
// ApplyCommand → Stage position2d 或 ROS cmd_vel
```

建议：`ApplyCommand` 内 `ApplyLimits` 后 `simulator_->Step`；Bridge 侧持有 Vehicle 注入 odom/TF。

### 8.6 参见
