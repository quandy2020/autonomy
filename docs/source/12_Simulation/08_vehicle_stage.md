# 8. 车辆抽象与 Stage

用 `VehicleInterface` 做限幅抽象，或理解 `map/conf/simulation/` 里 world 文件能/不能做什么。

`vehicle/` 提供底盘尺寸与 **KinematicsControl** 限幅；`ApplyCommand` 当前多仅缓存，需接 Gazebo/真机/sim 后端。

Stage 相关：**只有** world/yaml 资源（如 `cave.world`），**无**本仓库 C++ Stage 客户端读 `AUTONOMY_SIMULATION`。

| 目标 | 做法 |
|------|------|
| 测限幅逻辑 | vehicle 单测 + KinematicsControl |
| 2D 仿真 | 自接 Stage 或 Gazebo |
| 仅文件在仓库 | 勿当已集成 |

### VehicleInterface

文件：`vehicle/common/vehicle_inteface.hpp`（拼写以源码为准）。

| 方法 | 说明 |
|------|------|
| `Initialize(model)` | 尺寸、约束 |
| `GetVehicleInfo` | 状态查询 |
| `ApplyCommand` | 下发（待接后端） |

### KinematicsControl

Proto：`vehicle/proto/vehicle_options.proto`。字段含 `max_linear_speed`、`max_angular_speed`、加减速上限等。公式 [§3](03_math.md)。

### cave.world（示例资源）

| 元素 | 说明 |
|------|------|
| `pioneer2dx_with_laser` | 机器人模型 |
| `hokuyolaser` | 270° 激光 |
| floorplan | 约 16×16 m |

Player/Stage 格式；要跑需 **外部** Stage/Gazebo，不是 launch 自动加载。

### 常见坑

| 误解 | 事实 |
|------|------|
| Lua AUTONOMY_SIMULATION.stage | 无 SimulationServer 读取 |
| ApplyCommand 会动 Gazebo | 需 ROS cmd_vel 或自写后端 |

### 延伸阅读

[§5 架构](05_architecture.md)
