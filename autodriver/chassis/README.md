# Chassis（机器人本体硬件）

与 `autonomy/vehicle` **解耦**；消息体与 **`automsgs`** 一致。对外按**能力 + 统一运动/状态/安全**管理，对内按**运动学模型 + 厂商驱动**分化。

## 抽象（六概念）

| 概念 | 代码 | 作用 |
|------|------|------|
| **LocomotionModel** | `locomotion_model.hpp` | differential / omni / ackermann / legged / wheel_legged / humanoid + 限速与横向能力 |
| **MotionCommand** | `motion_command.hpp` | Twist + 可选 locomotion intent（stand/walk/wheel） |
| **BodyState** | `RobotState` | pose / twist / battery / flags；`active_cmd_id` = 运行模式名 |
| **SafetyGate** | `safety_gate.hpp` | 限速、去横向、原地转约束、看门狗；**先于**厂商 SDK |
| **Capability** | `capability.hpp` | 启动画像 JSON（能力通道），下游按 flags 编程 |
| **OperationalMode** | `operational_mode.hpp` | Idle → Armed → Moving → Docking / Fault / EStop |

作业附件（刷盘、刀盘、货箱门）走 **tool 通道**，不进 `cmd_vel`。

## 通道

| 方向 | Autolink 类型 | 默认 / 说明 |
|------|---------------|-------------|
| 运动指令 | `geometry_msgs.TwistStamped` | `/cmd_vel` |
| 模式指令 | `std_msgs.String` | `/chassis/mode`：`arm` `disarm` `estop` `clear_estop` `clear_fault` `dock` `undock` `stand` `walk` `wheel` |
| 工具指令 | `std_msgs.String` | 可选 `/chassis/tool`：`brush=1` |
| 状态 | `vehicle_msgs.RobotState` | `/robot_state`（`active_cmd_id`=mode） |
| 模式状态 | `std_msgs.String` | `/chassis/mode_state`：`armed,walk` |
| 能力 | `std_msgs.String` | `/chassis/capability` JSON |
| 事件 | `vehicle_msgs.RobotEvent` | FAULT / E-STOP / … |
| 里程计 | `nav_msgs.Odometry` | 由 RobotState 派生 |

```text
autonomy --Twist--> SafetyGate --> ChassisDriver
autonomy --mode/tool String--> ModeFSM / ApplyToolCommand
autonomy <--RobotState / Odom / Event / capability JSON--
```

## 分层

| 层 | 职责 |
|---|---|
| `ChassisDriver` | 厂商：`ApplyVelocityCommand` / `ApplyLocomotionIntent` / `ApplyToolCommand` / `ReadChassisState` |
| `ChassisBackendRegistry` | YAML `backend` → `CreateDriver` |
| `ChassisManager` | Autolink、Capability、Mode、SafetyGate；不链 `libautonomy` |
| `ChassisComponent` | 通用 `TimerComponent`；DAG 加载任意 backend 的 YAML |
| `stub/` | 无硬件差分积分；接受 intent / tool 日志 |
| `jetauto/` | 独立 `libautodriver_jetauto.so`：RRC + IK + `JetAutoComponent` |
| `l1w/` | 独立 `libautodriver_l1w.so`：钢镚 L1-W 全 HighLevel + `L1wComponent` |

## YAML

```yaml
chassis:
  enable: true
  name: base
  backend: stub
  locomotion: differential   # omni | ackermann | legged | wheel_legged | humanoid
  require_arm: false         # true：须先 arm 才接受 twist
  has_dock: false
  has_joint_bypass: false
  tools: [brush]             # 有 tool_cmd_channel 时广告
  max_linear_speed: 1.0
  max_angular_speed: 1.5
  min_turning_radius: 0.0
  # supports_lateral: true   # 覆盖 locomotion 默认
  cmd_vel_channel: /cmd_vel
  mode_cmd_channel: /chassis/mode
  mode_state_channel: /chassis/mode_state
  capability_channel: /chassis/capability
  tool_cmd_channel: ""       # 非空则启用
  state_channel: /robot_state
  event_channel: /robot_event
  odom_channel: /odom
  watchdog_ms: 200
  odom_period_ms: 20
```

## 加厂商

1. `chassis/<vendor>/driver.{hpp,cpp}` 实现 `ChassisDriver`
2. 需要时覆盖 `ApplyLocomotionIntent` / `ApplyToolCommand`
3. `REGISTER_CHASSIS_BACKEND(mybot, "mybot", CreateMyBotDriver);`
4. YAML：`backend: mybot` + 合适的 `locomotion` / `tools`

厂商只填 `RobotState` 硬件字段；Manager 写入模式名到 `active_cmd_id`。

## JetAuto（幻尔 / Hiwonder）

非 ROS：USB 串口直连 STM32 **ROS Robot Control Board**，主机协议见官方 wiki §3.14（帧 `0xAA 0x55`，`PACKET_FUNC_MOTOR=3`）。运动学对齐官方 `mecanum.py`。

| 项 | 值 |
|---|---|
| backend | `jetauto`（别名 `hiwonder`） |
| 库 | `libautodriver_jetauto.so`（`chassis/jetauto/CMakeLists.txt`） |
| Component | `autodriver::chassis::jetauto::JetAutoComponent` |
| 麦轮 | `locomotion: omni` + `params.drive_mode: mecanum` |
| 差分 | `locomotion: differential` + `params.drive_mode: differential`（`vy=0`） |
| 示例 | `config/chassis/jetauto.yaml` / `jetauto_differential.yaml` |
| DAG | `dag/chassis_jetauto.dag` |

```yaml
chassis:
  enable: true
  backend: jetauto
  locomotion: omni
  port: /dev/ttyACM0
  baud: 1000000
  params:
    drive_mode: mecanum   # or differential
    wheelbase: 0.216
    track_width: 0.195
    wheel_diameter: 0.097
```

`params.simulate: true` 可跳过串口（CI / 无硬件）。与 DualSense 共用 `/cmd_vel`。

## 钢镚 L1-W（智身 GENISOM / zsibot）

轮腿机型 **ZSL-1W**；实现全部在 `chassis/l1w/`（详见该目录 `README.md`），不扩展共享 `LocomotionIntent`。

| 项 | 值 |
|---|---|
| backend | `l1w`（别名 `genisom` / `zsibot` / `zsl-1w` / `l1-w`） |
| 库 | `libautodriver_l1w.so`（`chassis/l1w/CMakeLists.txt`） |
| Component | `autodriver::chassis::l1w::L1wComponent` |
| SDK | `autodriver/thirdparty/zsl1w` 或 `/opt/genisom_l1_sdk` |
| 示例 | `config/chassis/l1w.yaml` |
| DAG | `dag/chassis_l1w.dag` |

共享 mode：`stand` / `wheel`→move / `walk`→crawl。攀爬与特技仅 L1-W tools（`climb`、`shake_hand`、`rear_squat` 等）。

```yaml
chassis:
  enable: true
  backend: l1w
  locomotion: wheel_legged
  tools: [lie, passive, stand, cancel_crawl, cancel_climb, climb, crawl, move,
          attitude, shake_hand, rear_squat]
  tool_cmd_channel: /chassis/tool
  supports_lateral: "true"
  params:
    host: 192.168.168.168   # Wi-Fi: 192.168.234.1
    local_ip: 192.168.168.10
    auto_stand: true
    allow_lateral: true
    default_gait: move       # move | crawl | climb | stand
    sample_joints: true
    max_vx: 1.0
    max_vy: 0.5
    max_wz: 1.0
```
