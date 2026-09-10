# Chassis（机器人本体硬件）

与 `autonomy/vehicle` **解耦**：本目录只做真实底盘 / 本体 SDK；`vehicle` 留在 autonomy 进程内做运动学与模型抽象。

```text
autonomy  --/cmd_vel-->  ChassisManager  --> ChassisDriver (vendor SDK)
autonomy  <--/odom----  ChassisManager  <-- GetState()
```

## 分层

| 层 | 职责 |
|---|---|
| `ChassisDriver` | 厂商插件：`ApplyCommand` / `GetState` |
| `ChassisBackendRegistry` | YAML `backend` → 工厂 |
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
  odom_channel: /odom
  watchdog_ms: 200
  max_linear_speed: 1.0
  max_angular_speed: 1.5
```

## 与传感的关系

- 传感：`SensorManager` + `SensorDriver`（单向采样）
- 本体：`ChassisManager` + `ChassisDriver`（双向：指令进、状态出）
- 同属 `autodriver` 进程，共享 Autolink Node；互不 `#include` 对方 SDK
