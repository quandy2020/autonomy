# 使用方式

## 环境变量

| 变量 | 含义 |
|---|---|
| `AUTODRIVER_PATH` | 包根（其下有 `config/`） |
| `AUTODRIVER_DISTRIBUTION_HOME` | 安装树回退根 |
| `AUTODRIVER_PLUGIN_DIR` | 外置 `.so`（YAML `plugin_dir` 优先） |
| `LD_LIBRARY_PATH` | 须含 `build/lib`（`libautodriver` + Autolink） |
| `PATH` | 须含 `build/bin` |
| `AUTOLINK_PATH` / `AUTOLINK_LAUNCH_PATH` | Autolink 与 launch 目录 |
| `GLOG_logtostderr` | `1` 打日志到终端 |

## 1. 进程 `autodriver`

```bash
export AUTODRIVER_PATH=/path/to/autodriver   # 开发常为 …/src/autonomy/autodriver
export LD_LIBRARY_PATH=$BUILD/lib:$LD_LIBRARY_PATH
export PATH=$BUILD/bin:$PATH
autodriver                                          # 默认 autodriver_hardware.yaml
autodriver /path/to/autodriver                      # 指定配置根
autodriver /path/to/autodriver autodriver_hardware.yaml
```

参数：`[configuration_directory] [configuration_file]`。

启动序：`LoadConfig` → `Publisher::Initialize` → `SensorManager::{SetSink,Initialize,Start}` → `PoseFeeder::Start`（有 `compensator.pose_channel`）→ 等信号 → `Stop`。

只用的设备设 `enable: true`。相机折叠：`streams`/`point_clouds`/`imu`（见 [配置](configuration.md#camera)）。

### SDK 安装（包根 `scripts/`）

| 脚本 | 用途 |
|---|---|
| `install_rplidar_sdk.sh` | Slamtec SDK → 默认 `/usr/local` |
| `install_livox_sdk2.sh` / `install_livox_sdk.sh` | Livox SDK2 / SDK1 |
| `create_udev_rules.sh` / `delete_udev_rules.sh` | `/dev/rplidar` |
| `verify_realsense_d455.sh` | D455 冒烟（若有） |

CMake：`AUTODRIVER_WITH_{REALSENSE,ORBBEC,RPLIDAR,LIVOX}`。未找到 SDK → 对应 Create 为 nullptr。

## 2. Launch

```bash
export AUTOLINK_LAUNCH_PATH=$AUTODRIVER_PATH/launch
autolink launch start autodriver.launch
autolink launch list
autolink launch stop autodriver.launch
```

`launch/autodriver.launch`：`exception_handler: respawn`（最多重启 3 次）。自定义：

```xml
<process_name>autodriver /path/to/autodriver autodriver_hardware.yaml</process_name>
```

## 3. 嵌入库

```cpp
#include "autodriver/bridge/publisher.hpp"
#include "autodriver/config_loader.hpp"
#include "autodriver/sensor_manager.hpp"

auto config = autodriver::LoadConfig(dir, "autodriver_hardware.yaml");
autodriver::bridge::Publisher publisher(config.node_name);
publisher.Initialize();
autodriver::SensorManager manager(config);
manager.SetSink(&publisher);
manager.Initialize();
manager.Start();
// manager.Attach("imu/torso"); manager.Detach(...);
manager.Stop();
```

自定义总线：实现 `SampleSink`，不必用 `Publisher`。手写 Config：`examples/demo_main.cpp` → `autodriver_demo`。

运行时 API：`PushLidarPose` / `SetLidarPoseLookup`（补偿）；`HandleDeviceEvent`（测 udev）。

## 4. 约定

| 项 | 规则 |
|---|---|
| `enable: false` | 不进 Config，无法 Attach |
| `params_file` | 相对 `config/`；条目内 `params` 覆盖文件 |
| id | `name` 无 `/` 时加前缀（如 `lidar/`+`front`） |
| 折叠展开 id | `camera/<dev>_<stream>`、`camera/<dev>_points`、`imu/<dev>_imu` |

## 5. 故障速查

| 现象 | 处理 |
|---|---|
| 找不到 YAML | 查 `AUTODRIVER_PATH`（含 `config/` 的父目录） |
| `no enabled sensors` | 打开 `enable: true` |
| Create nullptr | 装 SDK / 看 CMake STATUS |
| Publisher 失败 | Autolink 环境、`AUTOLINK_PATH` |
| 串口/CAN 权限 | `dialout`；`ip link set can0 up …` |
| 无点云/无图 | 网段/UDP 端口；USB3；防火墙 |

系统化问答：[FAQ](../faq.md)。验证：[测试](testing.md)。
