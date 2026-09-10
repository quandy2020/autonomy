# Autodriver 文档

Autodriver 是 autonomy 栈中的传感器采集库：YAML 配置 → 内置 `SensorModule` 采集 → 可选 `SensorHub` 时间对齐 → `bridge::Publisher` 发布到 Autolink 通道。

工程布局：**按模态分包** + `common::Stream` / `canbus` 传输与协议层，同时保留统一 `SensorManager` 与 udev 热插拔。骨架模块（radar / microphone / smartereye）已注册，真采集待 ProtocolData / SDK。

## 阅读路径

1. [快速开始](guide/quickstart.md) — 构建、`autodriver`、launch  
2. [使用方式](guide/usage.md) — 进程 / launch / 嵌入库  
3. [架构与模块化](guide/architecture.md) — 分层、Registry、如何加厂商  
4. [数据流](guide/dataflow.md) — 相机 / 激光 / 串口路径  
5. [传感器手册](sensor/index.md) — 各厂商简介与用法  
6. [配置](guide/configuration.md) — YAML 字段  
7. [生命周期](guide/lifecycle.md) — Attach / Detach / udev  
8. [后端](guide/backends.md) — backend 与扩展  
9. [测试](guide/testing.md) — ctest 与用例表  
10. [API 概览](api/overview.md) — `Config`、`SensorManager`、注册表  
11. [FAQ](faq.md)

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

模块化要点：**Module 按模态固定，Driver 按厂商注册**；新增厂商一般只加 `camera|lidar/<vendor>/` + `REGISTER_*_BACKEND`，不必改 Manager。

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
  lidar/        # velodyne/、hesai/、livox/、rplidar/、stubs
  bridge/
```

## 常用命令

在 autonomy **仓库根目录**：

```bash
export AUTODRIVER_PATH=$PWD/src/autonomy/autodriver
export LD_LIBRARY_PATH=$PWD/build/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/bin:$PATH

./build/bin/autodriver
ctest --test-dir build -R autodriver --output-on-failure
```

包级说明见 [`README.md`](../../README.md)。
