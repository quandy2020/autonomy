# 测试

GTest；CMake `AUTODRIVER_BUILD_TEST=ON`；`autodriver_add_test` → `add_test(NAME …)`。

## 命令

```bash
# 仓库根
cmake -S . -B build -DBUILD_AUTODRIVER=ON -DAUTODRIVER_BUILD_TEST=ON
cmake --build build -j"$(nproc)" --target autodriver
ctest --test-dir build -R 'test_(config|config_loader|manager|hub|lidar|camera|realsense|nmea|wit|canbus|pose|compensator|stream|skeleton|time)' --output-on-failure

# 单二进制（路径以本机构建为准：build/bin 或 build/autonomy/bin）
./build/bin/test_config_loader
```

## 用例（与 CMakeLists 一致）

| 目标 | 文件 | 覆盖 |
|---|---|---|
| `test_time_sync` | `test_time_sync.cpp` | 时间同步 |
| `test_hub` | `test_hub.cpp` | SensorHub 对齐 |
| `test_nmea0183` | `test_nmea0183.cpp` | NMEA GGA/RMC |
| `test_wit_motion` | `test_wit_motion.cpp` | WitMotion 解析 |
| `test_realsense` | `test_realsense.cpp` | RealSense 辅助（可无设备） |
| `test_config` | `test_config.cpp` | Config / 查重 |
| `test_config_loader` | `test_config_loader.cpp` | YAML、params_file、**相机折叠**、扁平兼容 |
| `test_manager` | `test_manager.cpp` | Attach/Detach |
| `test_stream_lidar_base` | `test_stream_lidar_base.cpp` | Stream / Lidar 基类 |
| `test_pose_feeder` | `test_pose_feeder.cpp` | Odometry→PushLidarPose |
| `test_compensator_calibration` | `test_compensator_calibration.cpp` | 补偿与校准 YAML |
| `test_lidar_backend_registry` | `test_lidar_backend_registry.cpp` | 3D Registry/alias |
| `test_camera_backend_registry` | `test_camera_backend_registry.cpp` | 相机 Registry |
| `test_chassis_backend_registry` | `test_chassis_backend_registry.cpp` | 底盘 Registry（stub） |
| `test_canbus_skeleton` | `test_canbus_skeleton.cpp` | FakeCan / Protocol |
| `test_skeleton_modules` | `test_skeleton_modules.cpp` | stub Module 可加载 |

## 写法

| 场景 | 做法 |
|---|---|
| Loader | 临时目录写 YAML，`setenv(AUTODRIVER_PATH)`，`LoadConfig(basename)` |
| Registry | 断言 `Has` / Create 非空或 stub nullptr |
| UDP 驱动 | `source_type: raw_packet` + `PushRawPacket`，勿绑真网卡 |
| CI | 不依赖 USB 相机 / 真雷达 |

## 硬件冒烟（手动）

| 设备 | 步骤 |
|---|---|
| RealSense | enable 折叠 camera；`verify_realsense_d455.sh` |
| RPLidar | `install_rplidar_sdk.sh` + udev；`lidar_2d.enable` |
| Livox | 改 `host_ip`/`lidar_ip`；装 SDK2；同网段 |
| Velodyne/Hesai | UDP `data_port` 通；校准 YAML 可选 |

文档站点：`pip install -r docs/requirements.txt` → `cd docs && mkdocs serve` 或 `cmake --build build --target docs`。
