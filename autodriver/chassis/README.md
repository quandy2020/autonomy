# Chassis（机器人本体硬件）

与 `autonomy/vehicle` **解耦**；消息体与 **`automsgs/msgs/vehicle_msgs`** 一致。

| 方向 | Autolink 类型 | 协议 |
|------|---------------|------|
| 指令 | `geometry_msgs.TwistStamped` | 与 `RobotState.twist` 同体 |
| 状态 | `vehicle_msgs.RobotState` | 本体快照 |
| 事件 | `vehicle_msgs.RobotEvent` | FAULT / E-STOP / BATTERY_LOW … |
| 里程计 | `nav_msgs.Odometry` | 由 `RobotState` 派生（导航用） |

```text
autonomy  --TwistStamped-->  ChassisManager  --> ChassisDriver (vendor SDK)
autonomy  <--RobotState----  ChassisManager  <-- ReadChassisState()
autonomy  <--RobotEvent----  ChassisManager  <-- EmitChassisEvent()
```

## 分层

| 层 | 职责 |
|---|---|
| `ChassisDriver` | 厂商插件：`ApplyVelocityCommand` / `ReadChassisState` |
| `ChassisBackendRegistry` | YAML `backend` → `CreateDriver` |
| `ChassisManager` | Autolink IO、限速、看门狗；不链 `libautonomy` |
| `stub/` | 无硬件差分积分，联调用 |

## 加厂商

1. `chassis/<vendor>/driver.{hpp,cpp}` 实现 `ChassisDriver`
2. `REGISTER_CHASSIS_BACKEND(mybot, "mybot", CreateMyBotDriver);`
3. YAML：

```yaml
chassis:
  enable: true
  backend: mybot
  cmd_vel_channel: /cmd_vel
  state_channel: /robot_state
  event_channel: /robot_event
  odom_channel: /odom
  watchdog_ms: 200
```

厂商只填 `RobotState` 中硬件可知字段（pose / twist / battery / motion_enabled / dock / charge）；任务相关字段可由 autonomy 侧覆盖。
