# 5. 模块 Server

各子系统以 **Server** 为对外入口，由对应 `*_main` 构造运行。

## 5.1 一览

| Server | 类 | 典型进程 | 职责 |
|--------|-----|----------|------|
| 地图 | `map::MapServer` | planning 相关 | PGM/YAML → OccupancyGrid |
| 规划 | `planning::PlannerServer` | `autonomy.planning` | 全局规划、costmap |
| 控制 | `control::ControllerServer` | `autonomy.control` | FollowPath、`cmd_vel` |
| 变换 | `transform::TransformServer` | 各进程内 | 静态外参 → `tf_buffer` |
| 编排 | `task::TaskServer` | `autonomy.task` | BT 任务 |
| 车辆 | `vehicle::VehicleServer` | 底盘进程 | 限幅后下发 `VehicleInterface` |
| 音频 | `AudioComponent` | audio DAG | 麦阵检测 / 可选 ASR |

传感器对齐见 `sensor/` Collator；详表 [§9](09_sensor_vehicle_audio.md)。

## 5.2 要点

| Server | 关键 API / 说明 |
|--------|------------------|
| MapServer | `Start()`、`GetStaticMapShared()`、发布回调 |
| PlannerServer | `GetPlan`、`IsPathValid`、`GetCostmapWrapper` |
| ControllerServer | `Start()`、`GetLatestOdometry()`；costmap 进程内共享或 IPC |
| TransformServer | 静态外参；BT `TransformAvailable` 依赖缓冲 |
| SensorCollator | 激光等注入 Controller / costmap |

发令：Bridge / Action → `TaskServer` → BT → Planning / Control（跨进程）。

详见：[08 Planning](../08_Planning/01_architecture.md) · [09 Control](../09_Control/01_architecture.md) · [04 Running §3](../04_Running/03_autonomy_process.md)
