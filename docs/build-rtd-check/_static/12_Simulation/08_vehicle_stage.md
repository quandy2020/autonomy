# 8. 车辆抽象与 Stage

本文描述 `autonomy/vehicle` 车辆抽象层及 Stage 2D 仿真配置占位。

---

## 8.1 Vehicle 模块架构

```
VehicleServer
    │
    ▼
Vehicle (VehicleInterface 默认实现)
    │
    ├─ Initialize(model)     # 加载 VehicleModel
    ├─ ApplyCommand(cmd)     # 发送控制（待接仿真/硬件）
    └─ GetVehicleInfo(info)  # 查询状态
         │
         ▼
KinematicsControl::ApplyLimits(cmd)  # 速度/加速度限幅
```

---

## 8.2 VehicleInterface

**文件**：`autonomy/vehicle/common/vehicle_inteface.hpp`

| 方法 | 说明 |
|------|------|
| `Initialize(model)` | 加载尺寸、动力学约束 |
| `GetVehicleInfo(info)` | 返回速度、加速度等状态 |
| `ApplyCommand(cmd)` | 下发 `KinematicsControlCommand` |

**设计意图**：`ApplyCommand` 未来可对接：

- Gazebo 差速插件
- 真实底盘驱动
- `nav_test` 积分器

---

## 8.3 Vehicle 当前实现

```cpp
// vehicle.hpp 注释摘要
// ApplyCommand()：暂时只缓存最近一次控制指令，
// 实际硬件/仿真下发逻辑留作 TODO。
```

状态字段缓存在 `VehicleInfo` 中，供上层查询期望值。

---

## 8.4 KinematicsControl

**职责**：基于 `VehicleModel` 对 `Twist` 限幅：

| 约束 | 字段 |
|------|------|
| 最大线速度 | `max_linear_speed` |
| 最大倒车速度 | `max_reverse_speed` |
| 最大角速度 | `max_angular_speed` |
| 线/角加速度 | `max_linear_acceleration`, `max_angular_acceleration` |

**数学**：见 [03_math.md §3.6](03_math.md#36-速度限幅kinematicscontrol)。

统一处理差速、全向、Ackermann 的 $v_x, v_y, \omega_z$ 限幅。

---

## 8.5 VehicleModel（Proto）

`autonomy/vehicle/proto/vehicle_options.proto` 定义：

- 机器人尺寸（长、宽、高）
- 运动学类型（diff_drive / holonomic / ackermann）
- 速度、加速度、转向角上限

---

## 8.6 Stage 仿真配置

### 8.6.1 simulation.lua

```lua
AUTONOMY_SIMULATION = {
    stage = {
        world_file = "configuration_files/simulation/world/willow-erratic.world",
        map_file = "configuration_files/simulation/map/willow-erratic.yaml",
    },
    pedsim = { },
}
```

> 仓库内实际存在 `config/simulation/world/cave.world`，与 lua 中 `willow-erratic` 路径不一致。

### 8.6.2 cave.world 内容

| 元素 | 说明 |
|------|------|
| `pioneer2dx_with_laser` | 机器人实例 `robot_0` |
| `hokuyolaser` | 270° 2D 激光 |
| `camera_realsens_d435` | RGB-D 相机 |
| `floorplan "cave"` | 16×16 m 洞穴地图（`maps/cave.png`） |

Player/Stage 格式，可独立用 Stage 运行，**无**本仓库 C++ 客户端。

### 8.6.3 Pedsim 占位

`pedsim = {}` 预留行人仿真，未实现。

---

## 8.7 统一仿真后端（规划）

```cpp
class StageSimulator : public VehicleInterface {
    bool ApplyCommand(const KinematicsControlCommand& cmd) override {
        // 转发至 Stage position2d 接口
    }
};

class GazeboSimulator : public VehicleInterface {
    // 经 ROS topic 发布 cmd_vel
};
```

`SimulationServer` 根据 `simulation.lua` 选择后端实例化。

---

## 8.8 与 nav_test 的整合建议

将 `IntegrateDiffDrive` 移入 `Vehicle::ApplyCommand`：

```cpp
bool Vehicle::ApplyCommand(const KinematicsControlCommand& cmd) {
    KinematicsControl limits(model_);
    auto limited = cmd;
    limits.ApplyLimits(&limited);
    // 调用仿真后端积分
    simulator_->Step(limited, sim_dt_);
    return true;
}
```

`nav_test` 改为持有 `Vehicle` 而非直接调用 `IntegrateDiffDrive`。
