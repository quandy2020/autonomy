# Autodriver 文档

Autodriver 是 autonomy 栈中的传感器采集库：YAML 配置 → 内置 `SensorModule` 采集 → 可选 `SensorHub` 时间对齐 → `bridge::Publisher` 发布到 Autolink 通道。

工程布局：**按模态分包** + `common::Stream` / `canbus` 传输与协议层，同时保留统一 `SensorManager` 与 udev 热插拔。骨架模块（radar / microphone / smartereye）已注册，真采集待 ProtocolData / SDK。

## 阅读路径

1. [快速开始](guide/quickstart.md) — 构建、`autodriver`、launch
2. [配置](guide/configuration.md) — `autodriver_hardware.yaml` 字段说明
3. [生命周期](guide/lifecycle.md) — Attach / Detach / udev
4. [后端](guide/backends.md) — serial / canbus / RealSense / lidar / stub 模态
5. [API 概览](api/overview.md) — `Config`、`SensorManager`、`Publisher`、canbus / parser
6. [FAQ](faq.md) — 路径、权限、stub 与编译选项

## 架构概览

```
硬件 → Stream / SocketCAN / SDK → Parser / ProtocolData / Driver
                                    ↓
                              SensorModule（编入 libautodriver）
                                    ↓
                              SensorManager
                               ├─ SensorHub（可选对齐）
                               └─ SampleSink → bridge::Publisher → Autolink
```

源码树（节选）：

```
autodriver/autodriver/
  common/       # Stream、SerialPort、CanSocket、外参、status
  canbus/       # ProtocolData、Receiver、Client、Sender、byte
  imu/          # WitMotion serial/CAN
  gps/          # NMEA + gps/parser 工厂
  camera/       # realsense/、orbbec/、backend 注册表
  smartereye/   # camera backend stub
  radar/        # Conti stub + registry
  microphone/   # Respeaker stub + registry
  lidar/        # LidarComponentBase、packet_queue、scan_cut、velodyne/、hesai/、stubs
  bridge/
```

## 常用命令

在 autonomy **仓库根目录**：

```bash
export AUTODRIVER_PATH=$PWD/autodriver
export LD_LIBRARY_PATH=$PWD/build/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/bin:$PATH

./build/bin/autodriver
ctest --test-dir build -R autodriver --output-on-failure
```
