# 本体（chassis）

与传感同进程可选、与 `autonomy/vehicle` **解耦**。消息体为 automsgs（`TwistStamped` / `RobotState` / `RobotEvent`）。

完整抽象与加厂商步骤见包内 [`chassis/README.md`](../../../chassis/README.md)。

## 1. 两种启动方式

| 方式 | 何时用 | 注意 |
|---|---|---|
| **进程内** `ChassisManager` | `autodriver` + YAML `chassis.enable: true` | 与传感同进程；`backend: stub/jetauto/l1w` |
| **独立 Component** | `mainboard -d dag/chassis_*.dag` 或 launch | 独立 `.so`；**JetAuto 与 L1-W 二选一** |

推荐实机底盘：用 DAG / launch，主配置里保持 `chassis.enable: false`，避免双实例抢控制权。

### Launch（二选一）

[`launch/autodriver.launch`](../../../launch/autodriver.launch)：

1. binary `autodriver`（传感 / joy）
2. **一个** chassis `<module>`：`chassis_l1w.dag` **或** `chassis_jetauto.dag`

```bash
export AUTODRIVER_PATH=…/autodriver
export AUTOLINK_DAG_PATH=$AUTODRIVER_PATH/dag
export AUTOLINK_LIB_PATH=…/build/…/lib
export AUTOLINK_LAUNCH_PATH=$AUTODRIVER_PATH/launch
autolink launch start autodriver.launch
```

单独起底盘：

```bash
mainboard -d $AUTOLINK_DAG_PATH/chassis_l1w.dag
mainboard -d $AUTOLINK_DAG_PATH/chassis_jetauto.dag
```

## 2. 通道

| 方向 | 类型 | 默认 |
|---|---|---|
| 运动 | `TwistStamped` | `/cmd_vel` |
| 模式 | `String` | `/chassis/mode`：`arm` `estop` `stand` `walk` `wheel` … |
| 工具 | `String` | `/chassis/tool`（可选） |
| 状态 | `RobotState` | `/robot_state` |
| 模式状态 | `String` | `/chassis/mode_state` |
| 能力 | `String` JSON | `/chassis/capability` |
| 事件 | `RobotEvent` | `/robot_event` |
| 里程计 | `Odometry` | `/odom` |

## 3. Backend

| backend | 别名 | 库 | 配置 / DAG |
|---|---|---|---|
| `stub` | `sim` | `libautodriver.so` | YAML |
| `jetauto` | `hiwonder` | `libautodriver_jetauto.so` | `config/chassis/jetauto.yaml` · `dag/chassis_jetauto.dag` |
| `l1w` | `genisom` `zsibot` `zsl-1w` | `libautodriver_l1w.so` | `config/chassis/l1w.yaml` · `dag/chassis_l1w.dag` |

### JetAuto

幻尔 RRC USB 串口；麦轮 `locomotion: omni` + `drive_mode: mecanum`；差分 `differential`。`params.simulate: true` 可无串口。

### L1-W（钢镚 ZSL-1W）

| Mode / Tool | HighLevel |
|---|---|
| `stand` / tool `stand` | `standUp` |
| `wheel` + `/cmd_vel` | `move` |
| `walk` + `/cmd_vel` | `crawl` |
| `estop` / tool `passive` | `passive` |
| tool `lie` / `cancel_crawl` / `cancel_climb` / `climb` / `attitude` / `shake_hand` / `rear_squat` | 对应 API（均在 `chassis/l1w`） |

有线默认 `host=192.168.168.168`（Wi‑Fi 热点多为 `192.168.234.1`）。SDK：`autodriver/thirdparty/zsl1w` 或 `-DGenisomL1w_ROOT=`。详见 [`chassis/l1w/README.md`](../../../chassis/l1w/README.md)。

## 4. 手柄

根级 `joy:`（默认 DualSense）：按住 **L1** + 左摇杆 → `/cmd_vel`，与底盘共用通道。

| 步骤 | 命令 / 配置 |
|---|---|
| 蓝牙重新配对 | `autodriver --pair-joy`（或 `--pair-mode bluetooth`） |
| USB / 驱动挂载 | `autodriver --pair-joy --pair-mode usb`（别名 `driver` / `wired`） |
| 遥操 | YAML `joy.enable: true`，`device: /dev/input/js0`（以实际节点为准） |

完整字段、轴键映射与配对说明见 [配置 · joy](configuration.md#41-手柄遥操joy默认索尼-dualsenseps5)；CLI 见 [使用 · --pair-joy](usage.md#21-dualsense-pair-joy)。

## 5. 相关

| 页 | 内容 |
|---|---|
| [配置 · chassis](configuration.md) | YAML 字段 |
| [后端 · chassis](backends.md) | Registry |
| [使用 · Launch](usage.md) | 环境变量 |
| [FAQ](../faq.md) | DAG / SDK 排障 |
