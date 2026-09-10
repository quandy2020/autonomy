# 测试

单元测试用 GoogleTest，目标编入 autonomy `build`，前缀多为 `test_`。

## 运行

在 autonomy **仓库根目录**：

```bash
cmake -S . -B build -DBUILD_AUTODRIVER=ON -DAUTODRIVER_BUILD_TEST=ON
cmake --build build -j"$(nproc)" --target autodriver
# 或直接构建各 test_* 目标

ctest --test-dir build -R autodriver --output-on-failure
# 更细过滤：
ctest --test-dir build -R 'test_(config|config_loader|manager|hub|lidar|realsense|nmea|wit|canbus)' --output-on-failure
```

单测二进制通常在 `build/bin/` 或 `build/autonomy/bin/`（以工程 CMake 输出为准）：

```bash
./build/bin/test_config_loader
./build/bin/test_lidar_backend_registry
```

## 用例一览

| 测试 | 覆盖 |
|---|---|
| `test_config` | `Config` 结构、查重等 |
| `test_config_loader` | YAML 加载、channel 数组、`params_file`、**相机折叠展开**、扁平兼容 |
| `test_manager` | `SensorManager` Attach/Detach 生命周期 |
| `test_hub` | `SensorHub` 对齐窗口 |
| `test_time_sync` | 时间同步相关 |
| `test_lidar_backend_registry` | 3D lidar 注册表 Create / alias |
| `test_camera_backend_registry` | 相机 backend 注册 |
| `test_stream_lidar_base` | Stream / LidarComponentBase 钩子 |
| `test_compensator_calibration` | 运动补偿与校准 YAML |
| `test_pose_feeder` | Odometry → `PushLidarPose` |
| `test_nmea0183` | NMEA 解析 |
| `test_wit_motion` | WitMotion IMU 解析 |
| `test_canbus_skeleton` | CAN Client / Fake / Protocol 骨架 |
| `test_realsense` | RealSense 辅助逻辑（无设备时可部分跳过） |
| `test_skeleton_modules` | Radar / Mic 等 stub 模块可加载 |

## 编写建议

1. **配置类**：用临时目录写 YAML，`setenv("AUTODRIVER_PATH", …)`，调用 `LoadConfig(basename)`（参考 `test_config_loader.cpp`）。  
2. **Registry**：只断言工厂存在 / stub 返回 nullptr，不强制真实硬件。  
3. **驱动回放**：Velodyne/Hesai 用 `source_type: raw_packet` + `PushRawPacket`，避免依赖网卡。  
4. **勿**在 CI 默认依赖 USB 相机 / 真实雷达；硬件测放在脚本或手动 job。

## 硬件冒烟（手动）

| 设备 | 建议 |
|---|---|
| RealSense | `scripts/verify_realsense_d455.sh`（若有）；主配置 enable 后跑 `autodriver` |
| RPLidar | 装 SDK + udev；`lidar_2d.enable: true` |
| Livox | 同网段；改 `mid360.yaml` 的 `host_ip`/`lidar_ip` |

## 相关

- [快速开始](quickstart.md)
- [使用方式](usage.md)
- [FAQ](../faq.md)
