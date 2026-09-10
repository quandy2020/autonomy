# 使用方式

三种常见用法：独立进程、Autolink launch、嵌入其它程序。

## 1. 独立进程 `autodriver`

```bash
export AUTODRIVER_PATH=/path/to/autodriver   # 目录下有 config/
export LD_LIBRARY_PATH=.../build/lib:$LD_LIBRARY_PATH
export PATH=.../build/bin:$PATH

autodriver
# 或指定配置根与文件名
autodriver /path/to/autodriver autodriver_hardware.yaml
```

进程内顺序（`main.cpp`）：

1. `LoadConfig(dir, file)`  
2. `Publisher::Initialize()`  
3. `SensorManager::SetSink` → `Initialize` → `Start`  
4. `PoseFeeder::Start`（若配置了 `compensator.pose_channel`）  
5. 等待 SIGINT/SIGTERM → `Stop`

编辑 `config/autodriver_hardware.yaml`：只用的设备 `enable: true`。

### 相机折叠

一台物理机一条 `camera` 条目（`streams` / `point_clouds` / `imu`），详见
[配置 · camera](configuration.md#camera) 与 [RealSense](../sensor/camera/realsense.md)。

### 厂商 SDK

| 设备 | 安装 |
|---|---|
| RPLidar | `./scripts/install_rplidar_sdk.sh` |
| Livox | `./scripts/install_livox_sdk2.sh` / `install_livox_sdk.sh` |
| RealSense / Orbbec | 系统包或官方 SDK，CMake `find_package` |

未找到 SDK 时对应 backend stub，进程可启动但该传感器 Attach 失败并打日志。

---

## 2. Autolink launch

```bash
export AUTODRIVER_PATH=...
export AUTOLINK_LAUNCH_PATH=$AUTODRIVER_PATH/launch
autolink launch start autodriver.launch
autolink launch list
autolink launch stop autodriver.launch
```

`launch/autodriver.launch` 默认 `exception_handler: respawn`。自定义参数：

```xml
<process_name>autodriver /path/to/autodriver autodriver_hardware.yaml</process_name>
```

---

## 3. 嵌入库（C++）

链接 `autodriver`，不启动 `main`：

```cpp
#include "autodriver/bridge/publisher.hpp"
#include "autodriver/config_loader.hpp"
#include "autodriver/sensor_manager.hpp"

autodriver::Config config = autodriver::LoadConfig(dir, "autodriver_hardware.yaml");
// 或手写 config.sensors.push_back(...)

autodriver::bridge::Publisher publisher(config.node_name);
publisher.Initialize();

autodriver::SensorManager manager(config);
manager.SetSink(&publisher);
manager.Initialize();
manager.Start();

// 运行中：manager.Attach("imu/torso"); manager.Detach(...);

manager.Stop();
```

自定义发布：实现 `SampleSink`，在 `OnSample` 中写自有总线，不必用 `Publisher`。

手写最小配置示例：`examples/demo_main.cpp`（`autodriver_demo`）。

---

## 4. 配置约定速查

| 项 | 约定 |
|---|---|
| 配置根 | `AUTODRIVER_PATH` 指向含 `config/` 的包根 |
| 默认文件 | `autodriver_hardware.yaml` |
| 厂商参数 | `params_file: camera/...` 或 `lidar/...` |
| 仅 enable | `enable: false` 的条目**不会**进入 Config |
| 热插拔 | `hotplug.enable_udev` + `match`；serial 可自动填 tty |

---

## 5. 调试

```bash
export GLOG_logtostderr=1
autodriver
```

| 现象 | 排查 |
|---|---|
| `no enabled sensors` | YAML 全是 `enable: false` 或路径错 |
| backend Create nullptr | SDK 未装 / CMake 未找到 |
| 串口权限 | 用户加入 `dialout`；或 `create_udev_rules.sh` |
| 相机无图 | USB3、`rs-enumerate-devices` / Orbbec 工具 |

更多见 [FAQ](../faq.md)、[测试](testing.md)。
