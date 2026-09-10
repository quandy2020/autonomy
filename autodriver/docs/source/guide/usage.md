# 使用方式

构建与第一次跑通见 [快速开始](quickstart.md)。本页写：**进程参数、Launch、库嵌入、约定与排障**。

| 相关 | 链接 |
|---|---|
| 架构 / Registry | [架构](architecture.md) |
| 样本路径 | [数据流](dataflow.md) |
| YAML | [配置](configuration.md) |
| Attach / udev | [生命周期](lifecycle.md) |
| 系统化问答 | [FAQ](../faq.md) |

## 环境变量

| 变量 | 含义 |
|---|---|
| `AUTODRIVER_PATH` | 包根（其下有 `config/`）；开发常为 `$PWD/autodriver` |
| `AUTODRIVER_DISTRIBUTION_HOME` | 安装树回退根（未设 `AUTODRIVER_PATH` 时） |
| `AUTODRIVER_PLUGIN_DIR` | 外置插件 `.so` 目录；YAML `plugin_dir` **优先** |
| `LD_LIBRARY_PATH` | 须含 `build/lib`（`libautodriver` + Autolink + automsgs） |
| `PATH` | 须含 `build/bin`（`autodriver`） |
| `AUTOLINK_PATH` | Autolink 资源根 |
| `AUTOLINK_LAUNCH_PATH` | launch 目录，常为 `$AUTODRIVER_PATH/launch` |
| `GLOG_logtostderr` | `1` 日志打终端 |

`AUTODRIVER_PATH` 解析逻辑：`common/environment.hpp` → `WorkRoot()`；配置文件相对该根下的 `config/`。

## 1. 进程 `autodriver`

```bash
export AUTODRIVER_PATH=/path/to/autodriver   # 含 config/ 的包根
export LD_LIBRARY_PATH=$BUILD/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export PATH=$BUILD/bin${PATH:+:$PATH}

autodriver                                          # 默认 autodriver_hardware.yaml
autodriver /path/to/autodriver                      # 指定配置根，文件名仍默认
autodriver /path/to/autodriver autodriver_hardware.yaml
autodriver /path/to/autodriver camera/orbbec/gemini_330.yaml
```

| 参数 | 含义 |
|---|---|
| `argv[1]` | `configuration_directory`（包根或等价）；空则用 `AUTODRIVER_PATH` / 编译默认 |
| `argv[2]` | 配置 basename 或相对 `config/` 的路径；默认 `autodriver_hardware.yaml` |

### 启动 / 停止

```text
LoadConfig
  → Publisher::Initialize          // Autolink Node + SampleSink
  → SensorManager::{SetSampleSink, Initialize, Start}
  → PoseFeeder::Start              // compensator.pose_channel 空 → no-op 成功
  → ChassisManager::Start(node)    // chassis.enable=false → no-op 成功
  → 等 SIGINT/SIGTERM
  → Stop：Chassis → PoseFeeder → SensorManager
```

- 传感：只用的设备设 `enable: true`（或 legacy `attach_on_start`）。  
- 相机折叠：`streams` / `point_clouds` / `imu`（见 [配置 · camera](configuration.md)）。  
- 底盘：`chassis.enable: true` + `backend`（联调常用 `stub`）。  
- 正常日志含 `autodriver running (Ctrl+C to stop)`。

### SDK 安装（包根 `scripts/`）

| 脚本 | 用途 |
|---|---|
| `install_rplidar_sdk.sh` | Slamtec SDK → 默认 `/usr/local` |
| `install_livox_sdk2.sh` / `install_livox_sdk.sh` | Livox SDK2 / SDK1 |
| `create_udev_rules.sh` / `delete_udev_rules.sh` | `/dev/rplidar` |
| `verify_realsense_d455.sh` | D455 冒烟（若有） |

CMake：`AUTODRIVER_WITH_{REALSENSE,ORBBEC,RPLIDAR,LIVOX}`。未找到 SDK → 对应 backend Create 为 `nullptr`，不挡链接。装完后需重配 cmake 并确认 STATUS `… enabled`。

## 2. Launch

```bash
export AUTOLINK_LAUNCH_PATH=$AUTODRIVER_PATH/launch
autolink launch start autodriver.launch
autolink launch list
autolink launch stop autodriver.launch
```

| 项 | 说明 |
|---|---|
| 默认 process | `autodriver`（依赖环境里的 `AUTODRIVER_PATH`） |
| 崩溃策略 | `exception_handler: respawn`，`respawn_limit: 3` |
| 硬编码路径 | 改 launch 内 `<process_name>` |

```xml
<process_name>autodriver /path/to/autodriver autodriver_hardware.yaml</process_name>
```

Launch 文件注释见 `launch/autodriver.launch`。

## 3. 嵌入库

把 autodriver 链进自有进程时：自行 `autolink::Init` / `Clear`，并保证与 `libautodriver` 同构建树。

### 最小传感（YAML）

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

### 对齐进程入口（可选补偿 + 底盘）

与 `main.cpp` 同序时再加：

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

共享 **同一个** `publisher.GetNode()`，避免多 Node。

### 自定义总线

实现 `SampleSink`（`HandleSensorAttach` / `HandleSensorSample` / `HandleSensorDetach`），不必用 `Publisher`。手写 `Config` 不读 YAML：见 `examples/demo_main.cpp` → `autodriver_demo`（注意 demo 未注册真 backend 时 Create 可能为空，仅演示编排）。

### 运行时 API

| API | 用途 |
|---|---|
| `PushLidarPose` / `SetLidarPoseLookup` | 运动补偿；驱动须为 `MotionPoseSink` |
| `HandleDeviceEvent` | 模拟 / 注入 udev ADD/REMOVE |
| `SetAlignedCallback` / `SetRawSampleCallback` | Hub 旁路；`publish_aligned` 可自动把快照推 Sink |
| `ReportDiagnostic` | 推到 Sink → `/diagnostics` |
| `GetHub()` | 访问 `SensorHub` |
| `ChassisManager::GetDriver()` | 非拥有指针；未 Start / disable 为 `nullptr` |

链接：目标 `autodriver`；头文件 include 路径含包根与 `autodriver/` 子目录（与 CMake `target_include_directories` 一致）。

## 4. 约定

| 项 | 规则 |
|---|---|
| `enable: false` | **不进** `Config::sensors`，无法 Attach；udev 也不作用 |
| `autostart` | Start 时自动 Attach；可由 `enable` / `attach_on_start` 推导 |
| `params_file` | 相对 `config/`；条目内 `params` **覆盖**文件键 |
| id | `name` 无 `/` 时加模态前缀（如 `lidar/` + `front`） |
| 折叠展开 id | `camera/<dev>_<stream>`、`camera/<dev>_points`、`imu/<dev>_imu` |
| 内置 Module | `library` 空 → `libautodriver` 内 class_loader |
| 外置 Module | `library` 非空 → 按 `plugin_dir` / `AUTODRIVER_PLUGIN_DIR` 找 `.so` |
| 通道 | YAML `channel` 字符串或数组；空则 `ResolveChannel` |

## 5. 故障速查

| 现象 | 处理 |
|---|---|
| 找不到 YAML | `AUTODRIVER_PATH` = 含 `config/` 的包根（不是 `config` 本身） |
| `no enabled sensors` | 打开 `enable: true` |
| Create / driver `nullptr` | 装 SDK；看 cmake STATUS；`WITH_*=OFF` 则无该 backend |
| `CreateClassObj failed` | `module` 名是否为 `ImuModule` / `CameraModule` / …；是否链到当前 `libautodriver` |
| Publisher 失败 | Autolink 运行时、`AUTOLINK_PATH`、`LD_LIBRARY_PATH` |
| 串口 / CAN 权限 | 用户进 `dialout`；`ip link set can0 up type can bitrate …` |
| 无点云 / 无图 | UDP 端口与网段；USB3；防火墙；折叠项是否 enable |
| RPLidar A3 | `params_file: lidar/slamtec/a3.yaml`（256000） |
| 底盘无响应 | `chassis.enable`；`cmd_vel_channel`；watchdog 是否把速度清零 |

系统化问答：[FAQ](../faq.md)。验证：[测试](testing.md)。构建开关与路径：[快速开始](quickstart.md)。
