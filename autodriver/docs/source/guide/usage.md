# 使用方式

构建与首次运行见 [快速开始](quickstart.md)。本页说明进程外的用法：**环境变量、进程参数、Launch、库嵌入、约定与排障**。

> 排障表为常见现象与对应处理；是否复现取决于本机 SDK、权限与硬件，不构成运行验证清单。

| 相关 | 链接 |
|---|---|
| 架构 / Registry | [架构](architecture.md) |
| 样本路径 | [数据流](dataflow.md) |
| YAML | [配置](configuration.md) |
| Attach / udev | [生命周期](lifecycle.md) |
| 术语 | [术语](glossary.md) |
| 系统化问答 | [FAQ](../faq.md) |

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
```

短选项遵循 Google CLI 约定（`-h` help、`-n` dry-run；**不用** `-f`/`-n` 表示配置文件或节点名）。
`node_name` / `plugin_dir` / `compensator.pose_channel` 等运行时差异写在 YAML（或 `AUTODRIVER_PLUGIN_DIR`），不进命令行。

| 参数 | 含义 |
|---|---|
| `-h` / `--help` | 帮助 |
| `-V` / `--version` | 版本（来自 `version.json` → `conf/conf.hpp`） |
| `-n` / `--dry-run` | 加载配置后退出，不启硬件 |
| `--no-udev` | 关闭 udev 热插拔 |
| `-c` / `--config-dir` / 位置参数 | 配置根（含 `config/`）；亦读 `AUTODRIVER_PATH` |
| `--config-file` / 位置参数 | basename；默认 `autodriver_hardware.yaml`（**无**短选项 `-f`） |

### 2.1 启动 / 停止

与 `main.cpp` 一致：

```text
LoadConfig
  → Publisher::Initialize          // Autolink Node + SampleSink
  → SensorManager::{SetSampleSink, Initialize, Start}
  → PoseFeeder::Start              // compensator.pose_channel 空 → no-op 成功
  → ChassisManager::Start(node)    // chassis.enable=false → no-op 成功
  → 等待 SIGINT / SIGTERM
  → Stop：Chassis → PoseFeeder → SensorManager
```

| 要点 | 说明 |
|---|---|
| 传感 | 仅启用实际设备（`enable: true`，或旧别名 `attach_on_start`） |
| 相机 | 推荐折叠 `streams` / `point_clouds` / `imu`（见 [配置 · camera](configuration.md)） |
| 底盘 | `chassis.enable: true` 并指定 `backend`（联调阶段常用 `stub`） |
| 运行标志 | 日志含 `autodriver running (Ctrl+C to stop)` |

### 2.2 SDK 安装（包根 `scripts/`）

| 脚本 | 用途 |
|---|---|
| `install_rplidar_sdk.sh` | Slamtec SDK → 默认 `/usr/local` |
| `install_livox_sdk2.sh` / `install_livox_sdk.sh` | Livox SDK2 / SDK1 |
| `create_udev_rules.sh` / `delete_udev_rules.sh` | `/dev/rplidar` |
| `verify_realsense_d455.sh` | D455 冒烟（若有） |

CMake：`AUTODRIVER_WITH_{REALSENSE,ORBBEC,RPLIDAR,LIVOX}`。未找到 SDK 时对应 Create 返回 `nullptr`，不阻碍链接。安装后须重新 cmake，并确认 STATUS 出现 `… enabled`。开关表见 [快速开始 §2.1](quickstart.md#21-cmake-开关)。

---

## 3. Launch

```bash
export AUTOLINK_LAUNCH_PATH=$AUTODRIVER_PATH/launch
autolink launch start autodriver.launch
autolink launch list
autolink launch stop autodriver.launch
```

| 项 | 说明 |
|---|---|
| 默认 process | `autodriver`（依赖环境中的 `AUTODRIVER_PATH`） |
| 崩溃策略 | `exception_handler: respawn`，`respawn_limit: 3` |
| 硬编码路径 | 修改 launch 内 `<process_name>` |

```xml
<process_name>autodriver /path/to/autodriver autodriver_hardware.yaml</process_name>
```

文件注释见 `launch/autodriver.launch`。

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

系统化问答见 [FAQ](../faq.md)。验证见 [测试](testing.md)。
