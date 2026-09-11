# 测试

本页说明如何构建并运行 autodriver 的 GTest 用例，以及编写与硬件冒烟的约定。

| 项 | 事实 |
|---|---|
| 框架 | GTest |
| 开关 | `AUTODRIVER_BUILD_TEST=ON` |
| 注册 | `autodriver_add_test` → `add_test(NAME …)`（见 `autodriver/CMakeLists.txt`） |

> 用例表与 CMake 中的测试目标对齐（配置/源码确认）。下列命令是否在本机通过，须以实际 `ctest` 结果为准。

| 相关 | 链接 |
|---|---|
| 构建总览 | [快速开始](quickstart.md) |
| 排障 | [FAQ](../faq.md) |
| 术语 | [术语](glossary.md) |

---

## 1. 命令

在 **autonomy 仓库根**执行：

```bash
cmake -S . -B build -DBUILD_AUTODRIVER=ON -DAUTODRIVER_BUILD_TEST=ON
cmake --build build -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)" --target autodriver
ctest --test-dir build -R 'test_(config|config_loader|manager|hub|lidar|camera|realsense|nmea|wit|canbus|pose|compensator|stream|skeleton|time)' --output-on-failure
```

单二进制（路径以本机构建为准：`build/bin` 或 `build/autonomy/bin`）：

```bash
./build/bin/test_config_loader
```

---

## 2. 用例一览

源文件均在 `autodriver/test/`（与下表文件名对应）。

### 2.1 配置与编排

| 目标 | 文件 | 覆盖 |
|---|---|---|
| `test_config` | `test_config.cpp` | Config / id 查重 |
| `test_config_loader` | `test_config_loader.cpp` | YAML、`params_file`、相机折叠、扁平兼容 |
| `test_manager` | `test_manager.cpp` | Attach / Detach |

### 2.2 对齐、补偿与时间

| 目标 | 文件 | 覆盖 |
|---|---|---|
| `test_hub` | `test_hub.cpp` | SensorHub 对齐 |
| `test_time_sync` | `test_time_sync.cpp` | 时间同步 |
| `test_pose_feeder` | `test_pose_feeder.cpp` | Odometry → `PushLidarPose` |
| `test_compensator_calibration` | `test_compensator_calibration.cpp` | 运动补偿与校准 YAML |

### 2.3 解析与厂商辅助

| 目标 | 文件 | 覆盖 |
|---|---|---|
| `test_nmea0183` | `test_nmea0183.cpp` | NMEA GGA / RMC |
| `test_wit_motion` | `test_wit_motion.cpp` | WitMotion 解析 |
| `test_realsense` | `test_realsense.cpp` | RealSense 辅助（可无设备） |
| `test_stream_lidar_base` | `test_stream_lidar_base.cpp` | Stream / Lidar 基类 |

### 2.4 Registry 与占位

| 目标 | 文件 | 覆盖 |
|---|---|---|
| `test_lidar_backend_registry` | `test_lidar_backend_registry.cpp` | 3D Registry / alias |
| `test_camera_backend_registry` | `test_camera_backend_registry.cpp` | 相机 Registry |
| `test_chassis_backend_registry` | `test_chassis_backend_registry.cpp` | 底盘 Registry（stub） |
| `test_canbus_skeleton` | `test_canbus_skeleton.cpp` | FakeCan / Protocol |
| `test_skeleton_modules` | `test_skeleton_modules.cpp` | stub Module 可加载 |

---

## 3. 编写约定

| 场景 | 做法 |
|---|---|
| Loader | 临时目录写 YAML，`setenv(AUTODRIVER_PATH)`，再 `LoadConfig(basename)` |
| Registry | 断言 `Has`；Create 非空，或 stub 时为 `nullptr` |
| UDP 驱动 | `source_type: raw_packet` + `PushRawPacket`；请勿绑定真实网卡 |
| CI | 不依赖 USB 相机或真实雷达 |

---

## 4. 硬件冒烟（手动）

单元测试不替代实机确认。下列为常见手动步骤：

| 设备 | 步骤 |
|---|---|
| RealSense | 折叠 `camera` 设 `enable: true`；运行 `scripts/verify_realsense_d455.sh` |
| RPLidar | `install_rplidar_sdk.sh` + 可选 udev；`lidar_2d.enable: true` |
| Livox | 安装 SDK2；修改 `host_ip` / `lidar_ip`；保持同网段 |
| Velodyne / Hesai | 确保 UDP `data_port` 可达；校准 YAML 可选 |

---

## 5. 文档站点

```bash
pip install -r autodriver/docs/requirements.txt
cd autodriver/docs && mkdocs serve
# 或
cmake --build build --target docs
```
