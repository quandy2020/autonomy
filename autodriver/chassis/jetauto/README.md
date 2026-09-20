# JetAuto — Autolink Component + RRC chassis backend (non-ROS)

独立共享库 `libautodriver_jetauto.so`：厂商驱动 + `JetAutoComponent`。

## 产物

| Target | 输出 | 内容 |
|--------|------|------|
| `autodriver_jetauto` | `libautodriver_jetauto.so` | IK / RRC / `ChassisDriver` / `JetAutoComponent` |

CMake：本目录 `CMakeLists.txt`（由 `chassis/CMakeLists.txt` `add_subdirectory`）。

## Component（推荐）

```text
mainboard -d dag/chassis_jetauto.dag
```

| 项 | 值 |
|----|-----|
| `module_library` | `libautodriver_jetauto.so` |
| `class_name` | `autodriver::chassis::jetauto::JetAutoComponent` |
| `config_file_path` | `chassis/jetauto.yaml`（相对 `AUTODRIVER_PATH/config/`） |

可调参数写在 YAML（`port` / `baud` / `drive_mode` / 几何尺寸等），见 `config/chassis/jetauto.yaml`。

## 进程模式

`autodriver` 二进制已链接 `autodriver_jetauto`，YAML：

```yaml
chassis:
  enable: true
  backend: jetauto
  locomotion: omni          # or differential
  port: /dev/ttyACM0
  baud: 1000000
  params:
    drive_mode: mecanum     # or differential
    wheelbase: 0.216
    track_width: 0.195
    wheel_diameter: 0.097
    # simulate: true
```

## 源文件

| File | Role |
|------|------|
| `jetauto_component.*` | Autolink `TimerComponent` |
| `driver.*` | `ChassisDriver` + `REGISTER_CHASSIS_BACKEND` |
| `kinematics.*` | 麦轮 / 差分 IK |
| `rrc_protocol.*` | `0xAA 0x55` 主机协议 |

## 协议

- [ROS Robot Control Board wiki](https://wiki.hiwonder.com/projects/ROS-Robot-Control-Board/en/latest/) §3.14
- 运动学默认对齐 JetAuto `mecanum.py`
