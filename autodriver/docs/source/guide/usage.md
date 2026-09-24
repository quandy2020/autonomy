# 使用方式

构建与首次运行见 [快速开始](quickstart.md)。本页说明进程外的用法：**环境变量、进程参数、Launch、库嵌入、约定与排障**。

> 排障表为常见现象与对应处理；是否复现取决于本机 SDK、权限与硬件，不构成运行验证清单。

| 相关 | 链接 |
|---|---|
| 架构 / Registry | [架构](architecture.md) |
| 底盘 / DAG | [本体](chassis.md) |
| 样本路径 | [数据流](dataflow.md) |
| YAML | [配置](configuration.md) |
| Attach / udev | [生命周期](lifecycle.md) |
| FAQ | [FAQ](../faq.md) |

---

## 1. 环境变量

| 变量 | 含义 |
|---|---|
| `AUTODRIVER_PATH` | 包根（其下含 `config/`）；开发时通常为 `$PWD/autodriver` |
| `AUTODRIVER_DISTRIBUTION_HOME` | 安装树回退根（未设置 `AUTODRIVER_PATH` 时） |
| `AUTODRIVER_PLUGIN_DIR` | 外置插件 `.so` 目录；YAML `plugin_dir` **优先** |
| `LD_LIBRARY_PATH` | 须含 `build/lib`（`libautodriver`、Autolink、automsgs） |
| `PATH` | 须含 `build/bin`（`autodriver`） |
| `AUTOLINK_PATH` | Autolink 资源根 |
| `AUTOLINK_LAUNCH_PATH` | launch 目录，通常为 `$AUTODRIVER_PATH/launch` |
| `AUTOLINK_DAG_PATH` | DAG 目录，通常为 `$AUTODRIVER_PATH/dag` |
| `AUTOLINK_LIB_PATH` | Component `.so`（`libautodriver_jetauto.so` / `_l1w.so`） |
| `GLOG_logtostderr` | 设为 `1` 时日志输出至终端 |

`AUTODRIVER_PATH` 解析：`common/environment.hpp` → `WorkRoot()`；配置文件相对该根下的 `config/`。

---

## 2. 进程 `autodriver`

CLI（CLI11，见包根 `options.hpp` / `options.cpp`）：`autodriver --help` / `-V`。

```bash
export AUTODRIVER_PATH=/path/to/autodriver   # 含 config/ 的包根
export LD_LIBRARY_PATH=$BUILD/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export PATH=$BUILD/bin${PATH:+:$PATH}

autodriver --help
autodriver -V
autodriver                                          # 默认 autodriver_hardware.yaml
autodriver /path/to/autodriver                      # 指定配置根
autodriver -c /path/to/autodriver --config-file autodriver_hardware.yaml
autodriver -n                                       # dry-run：只加载配置
autodriver --no-udev
# DualSense 配对 / 等待 js*（不启传感与底盘，跑完即退出）
autodriver --pair-joy                               # 默认蓝牙
autodriver --pair-joy --pair-mode bluetooth --pair-timeout 60
autodriver --pair-joy --pair-mode usb               # USB / hid-playstation
autodriver --pair-joy --pair-mode driver            # usb 别名
```

短选项遵循 Google CLI 约定（`-h` help、`-n` dry-run；**不用** `-f`/`-n` 表示配置文件或节点名）。
`node_name` / `plugin_dir` / `compensator.pose_channel` 等运行时差异写在 YAML（或 `AUTODRIVER_PLUGIN_DIR`），不进命令行。

| 参数 | 含义 |
|---|---|
| `-h` / `--help` | 帮助 |
| `-V` / `--version` | 版本（来自 `version.json` → `conf/conf.hpp`） |
| `-n` / `--dry-run` | 加载配置后退出，不启硬件 |
| `--no-udev` | 关闭 udev 热插拔 |
| `--pair-joy` | DualSense 配对/等待后退出（见 `--pair-mode`） |
| `--pair-mode` | `bluetooth`/`bt`（默认）或 `usb`/`wired`/`driver` |
| `--pair-timeout` | `--pair-joy` 扫描/等待秒数（默认 45） |
| `-c` / `--config-dir` / 位置参数 | 配置根（含 `config/`）；亦读 `AUTODRIVER_PATH` |
| `--config-file` / 位置参数 | basename；默认 `autodriver_hardware.yaml`（**无**短选项 `-f`） |

### 2.1 DualSense 遥操

`joy.enable: true` 且 `profile: dualsense` 时，进程读 `/dev/input/js*`，以 `publish_hz`（默认 50 Hz）发布 `sensor_msgs/Joy` 和差速 `TwistStamped`（`linear.x`、`angular.z`）。摇杆回中也继续发零速。通道默认跟底盘 `cmd_vel_channel`（通常 `/cmd_vel`）。

日常连接不必先跑 `--pair-joy`。`bluetooth_connect: true`（该 profile 的默认）时，若 `device` 打不开，进程用 `bluetoothctl` 自己完成扫描、配对、信任和连接，然后打开出现的 `js` 节点。已经 `Bonded` 的手柄只重连，不删绑定。断连发零速并重试。

| 操作 | 轴 | 速度 |
|---|---|---|
| 左摇杆前推 / 后拉 | axis 1（上为负，已取反） | `linear.x` 正 / 负，满杆 `1.5` m/s |
| 右摇杆向右 / 向左 | axis 3（右为正，已取反） | `angular.z` 负 / 正，满杆 `1.5` rad/s |
| 松开摇杆 | — | 立即回零（`max_linear_acc` / `max_angular_acc` 为 `0`） |

默认 `require_enable: false`，不必按住 L1。`enable_button` 仍是 L1（button 4），只有把 `require_enable` 改成 `true` 才要按住才有速度。

axis 2 是 L2 扳机，松开时停在 `-1`。把它当成转向轴时，`angular.z` 会一直是 `-1.5`。hid-playstation 的轴序是：0 左 X、1 左 Y、2 L2、3 右 X、4 右 Y、5 R2。

蓝牙步骤：

1. 宿主机加载 `hid_playstation`（`modprobe hid_playstation`）。容器里没有这份模块文件，在宿主机加载即可；`/dev/input` 与宿主机共用。
2. 进程能调用 `bluetoothctl`（安装 bluez），并能看到本机适配器。容器需要挂载宿主机 D-Bus（`/run/dbus`）。
3. 按住 **Create + PS**，直到灯条快闪，再启动 `autodriver`。
4. 日志出现 `DualSense joystick ready` 和 `LinuxJoystick opened` 后即可推杆。
5. 配对成功后灯条熄灭、连接报 `Host is down`：短按一下 **PS** 唤醒。这时不要再按 Create + PS，那会重新进入配对。

`--pair-joy` 是一次性命令：在 `Run()` 之前执行，不加载传感配置、不启底盘和 JoyTeleop，做完就退出。它会**删掉**已配对的 DualSense 再重新配对。已经能连上时用正常启动即可。

| 模式 | 别名 | 行为 |
|---|---|---|
| `bluetooth`（默认） | `bt` | 移除旧 DualSense 绑定 → 扫描 → pair / trust / connect |
| `usb` | `wired`、`driver` | 提示插 USB 线；`modprobe hid_playstation`；等待 `/dev/input/js*` |

USB 插上后把出现的节点写入 `joy.device`，并设 `bluetooth_connect: false`。用户宜在 `input` 组。字段见 [配置 · joy](configuration.md#41-手柄遥操joy默认索尼-dualsenseps5)。

### 2.2 启动 / 停止

与 `main.cpp` 一致（`--pair-joy` 走独立路径，不经下列序列）：

```text
LoadConfig
  → Publisher::Initialize          // Autolink Node + SampleSink
  → SensorManager::{SetSampleSink, Initialize, Start}
  → PoseFeeder::Start              // compensator.pose_channel 空 → no-op 成功
  → ChassisManager::Start(node)    // chassis.enable=false → no-op 成功
  → JoyTeleop::Start(node)         // joy.enable=false → no-op；无 js 且 bluetooth_connect 时在发布循环里重连
  → 等待 SIGINT / SIGTERM
  → Stop：JoyTeleop → Chassis → PoseFeeder → SensorManager
```

| 要点 | 说明 |
|---|---|
| 传感 | 仅启用实际设备（`enable: true`，或旧别名 `attach_on_start`） |
| 相机 | 推荐折叠 `streams` / `point_clouds` / `imu`（见 [配置 · camera](configuration.md)） |
| 底盘（进程内） | `chassis.enable: true` + `backend`（联调常用 `stub`） |
| 底盘（实机） | 推荐独立 DAG / launch；主 YAML 保持 `chassis.enable: false`。见 [本体](chassis.md) |
| 手柄 | `joy.enable: true` + DualSense；蓝牙由 `bluetooth_connect` 在启动时完成，见 §2.1 |
| 运行标志 | 日志含 `autodriver running (Ctrl+C to stop)` |

### 2.3 SDK 安装（包根 `scripts/`）

| 脚本 | 用途 |
|---|---|
| `install_rplidar_sdk.sh` | Slamtec SDK → 默认 `/usr/local` |
| `install_livox_sdk2.sh` / `install_livox_sdk.sh` | Livox SDK2 / SDK1 |
| `install_realsense_sdk.sh` | librealsense2（apt / source） |
| `install_orbbec_sdk.sh` | OrbbecSDK v2（.deb / source） |
| `create_udev_rules.sh` / `delete_udev_rules.sh` | `/dev/rplidar` |
| `verify_realsense_d455.sh` | D455 冒烟（若有） |

CMake：`AUTODRIVER_WITH_{REALSENSE,ORBBEC,RPLIDAR,LIVOX}`。未找到 SDK 时对应 Create 返回 `nullptr`，不阻碍链接。安装后须重新 cmake，并确认 STATUS 出现 `… enabled`。开关表见 [快速开始 §2.1](quickstart.md#21-cmake-开关)。

---

## 3. Launch + 底盘 DAG

```bash
export AUTOLINK_LAUNCH_PATH=$AUTODRIVER_PATH/launch
export AUTOLINK_DAG_PATH=$AUTODRIVER_PATH/dag
export AUTOLINK_LIB_PATH=$BUILD/lib
autolink launch start autodriver.launch
autolink launch list
autolink launch stop autodriver.launch
```

| 项 | 说明 |
|---|---|
| 传感 | binary `autodriver`（`exception_handler: respawn`，limit 3） |
| 底盘 | **二选一**：`chassis_l1w.dag` 或 `chassis_jetauto.dag`（launch 内注释切换） |
| 单独起底盘 | `mainboard -d $AUTOLINK_DAG_PATH/chassis_l1w.dag` |

文件注释见 `launch/autodriver.launch`；通道与厂商见 [本体](chassis.md)。

---

## 4. 嵌入库

将 autodriver 链入自有进程时：自行调用 `autolink::Init` / `Clear`，并保证与 `libautodriver` 同构建树。

链接目标：`autodriver`。头文件 include 路径须含包根与 `autodriver/` 子目录（与 CMake `target_include_directories` 一致）。

### 4.1 最小传感（YAML）

```cpp
#include "autodriver/bridge/publisher.hpp"
#include "autodriver/config_loader.hpp"
#include "autodriver/sensor_manager.hpp"
#include "autolink/init.hpp"

autolink::Init(argv[0]);
auto config = autodriver::LoadConfig(dir, "autodriver_hardware.yaml");
autodriver::bridge::Publisher publisher(config.node_name);
if (!publisher.Initialize()) { /* … */ }
autodriver::SensorManager manager(config);
manager.SetSampleSink(&publisher);
if (!manager.Initialize() || !manager.Start()) { /* … */ }
// 运行中：
// manager.AttachSensor("imu/torso");
// manager.DetachSensor("imu/torso");
manager.Stop();
autolink::Clear();
```

### 4.2 对齐进程入口（可选补偿 + 底盘）

与 `main.cpp` 同序时再增加：

```cpp
#include "autodriver/bridge/pose_feeder.hpp"
#include "chassis/chassis_manager.hpp"

autodriver::bridge::PoseFeeder pose_feeder;
pose_feeder.Start(publisher.GetNode(), &manager, config);

autodriver::chassis::ChassisManager chassis;
chassis.Start(publisher.GetNode(), config);

// … 运行 …
chassis.Stop();
pose_feeder.Stop();
manager.Stop();
```

须共享**同一个** `publisher.GetNode()`，避免多 Node。

### 4.3 自定义总线

实现 `SampleSink`（`HandleSensorAttach` / `HandleSensorSample` / `HandleSensorDetach`）即可，不必使用 `Publisher`。

以代码构造 `Config`、不读取 YAML 的示例：`examples/demo_main.cpp`（目标 `autodriver_demo`）。demo 未注册真实 backend 时 Create 可能返回 `nullptr`，仅用于演示编排。

### 4.4 运行时 API

| API | 用途 |
|---|---|
| `PushLidarPose` / `SetLidarPoseLookup` | 运动补偿；驱动须为 `MotionPoseSink` |
| `HandleDeviceEvent` | 模拟 / 注入 udev ADD/REMOVE |
| `SetAlignedCallback` / `SetRawSampleCallback` | 对齐旁路；`publish_aligned` 可将快照推 Sink |
| `ReportDiagnostic` | 推到 Sink → `/diagnostics` |
| `GetHub()` | 访问 `SensorHub` |
| `ChassisManager::GetDriver()` | 非拥有指针；未 Start 或 disable 时为 `nullptr` |

更多头文件索引见 [API 概览](../api/overview.md)。

---

## 5. 约定

| 项 | 规则 |
|---|---|
| `enable: false` | **不进入** `Config::sensors`，无法 Attach；udev 亦不作用 |
| `autostart` | `Start` 时自动 Attach；可由 `enable` / `attach_on_start` 推导 |
| `params_file` | 相对 `config/`；条目内 `params` **覆盖**文件键 |
| id | `name` 无 `/` 时加模态前缀（如 `lidar/` + `front`） |
| 折叠展开 id | `camera/<dev>_<stream>`、`camera/<dev>_points`、`imu/<dev>_imu` |
| 内置 Module | `library` 空 → `libautodriver` 内 class_loader |
| 外置 Module | `library` 非空 → 按 `plugin_dir` / `AUTODRIVER_PLUGIN_DIR` 查找 `.so` |
| 通道 | YAML `channel` 字符串或数组；为空则经 `ResolveChannel` |

完整字段见 [配置](configuration.md)；enable / Attach 语义见 [生命周期](lifecycle.md)。

---

## 6. 故障排查

| 现象 | 处理 |
|---|---|
| 找不到 YAML | 将 `AUTODRIVER_PATH` 设为含 `config/` 的包根（不是 `config` 目录本身） |
| `no enabled sensors` | 将所需设备设为 `enable: true` |
| Create / driver 为 `nullptr` | 安装对应 SDK；查看 CMake STATUS；`WITH_*=OFF` 时无该 backend |
| `CreateClassObj failed` | 确认 `module` 名为 `ImuModule` / `CameraModule` 等，且链接至当前 `libautodriver` |
| Publisher 失败 | 检查 Autolink 运行时、`AUTOLINK_PATH`、`LD_LIBRARY_PATH` |
| 串口 / CAN 权限 | 将用户加入 `dialout`；执行 `ip link set can0 up type can bitrate …` |
| 无点云 / 无图像 | 检查 UDP 端口与网段、USB3、防火墙；折叠子项是否 enable |
| RPLidar A3 | 使用 `params_file: lidar/slamtec/a3.yaml`（波特率 256000） |
| 底盘无响应 | 确认 `chassis.enable`、`cmd_vel_channel`；检查看门狗是否将速度清零 |
| 手柄无 `/dev/input/js*` | Create + PS 至灯条快闪后再启动；或 `autodriver --pair-joy` / `--pair-mode usb`。宿主机 `modprobe hid_playstation`，用户加入 `input` 组 |
| `bluetoothctl not found` | 安装 bluez |
| 容器里 `bluetoothctl` 起不来 | 把宿主机 `/run/dbus` 挂进容器 |
| 已配对但 `Host is down`、仍无 `js*` | 短按 PS 唤醒后再连；不要按 Create + PS |
| `angular.z` 松开仍是 `-1.5` | 转向轴误用了 L2（axis 2）。设 `angular_axis: 3` |
| 推杆发迟、不跟手 | `max_linear_acc` 与 `max_angular_acc` 设为 `0` |
| 左右转反了 | `invert_angular: true`（杆向右为负 `angular.z`） |
| USB 超时 | 检查线缆、`lsusb \| grep Sony`、宿主机 `modprobe hid_playstation` |

系统化问答见 [FAQ](../faq.md)。验证见 [测试](testing.md)。
