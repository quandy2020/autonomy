# autodriver 对齐 Apollo drivers 工程模式 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 autodriver 向 Apollo `modules/drivers` 工程模式对齐：按模态/厂商分包、Stream×Parser 分离、Lidar Scan→Convert 骨架、诊断与强类型配置，同时保留统一 SensorManager 与 udev 热插拔。

**Architecture:** 单进程 `SensorManager` + 模态目录（`imu/`/`gps/`/`camera/`/`lidar/`）+ `common/`（Stream/Serial/CAN）+ `bridge::Publisher`。配置短期 YAML 兼容，proto schema 演进为运行时真源。

**Tech Stack:** C++17, CMake, yaml-cpp, autolink, automsgs, optional librealsense2/libudev

**Spec:** 对话结论（2026-09-05 Apollo drivers 对照）；保留既有 modular-hotplug 能力

## Global Constraints

- 始终中文回复用户；不主动 commit
- 不改 `.superpowers/`；本计划写入 `docs/superpowers/plans/`
- 保留 `SensorManager` / udev / 内置 modules；不引入 Cyber DAG
- include 路径与目录一致；少造轮子，优先复用 autolink
- 用户环境可不强制全量 build；改完后尽量保证 include/CMake 自洽

---

## File map（目标树）

```
autodriver/autodriver/
  common/
    environment.hpp
    serial_port.{hpp,cpp}      # from hardware/
    can_socket.{hpp,cpp}       # from hardware/
    stream.{hpp,cpp}           # NEW：传输抽象
  imu/
    wit_motion_parser.*
    serial_imu_driver.*
    can_imu_driver.*
  gps/
    nmea_0183.*
    serial_gps_driver.*
    can_gps_driver.*
  camera/
    realsense_*.*
  lidar/
    lidar_component_base.hpp   # NEW：Scan/PointCloud + SourceType
    source_type.hpp            # NEW
  common/status.hpp           # 设备健康（原 diagnostics/）
  modules.cpp                  # MakeDriver 分支；include 新路径
  bridge/ …
```

---

### Task 1: 按模态拆分 hardware/ 目录

**Deliverable:** `hardware/` 迁入 `common|imu|gps|camera/`，全部 include/CMake/测试路径更新，无残留 `hardware/` 引用。

- [x] 创建目标目录并用 `git mv` 移动源文件
- [x] 批量更新 `#include "autodriver/hardware/..."` 与 include guard
- [x] 更新 `autodriver/CMakeLists.txt` 源列表与 RealSense 条件源
- [x] 更新 `modules.cpp`、测试 include
- [x] 删除空 `hardware/` 目录

---

### Task 2: Stream 传输抽象（对齐 Apollo GNSS Stream）

**Deliverable:** `common/stream.hpp` + Serial 实现；`SerialGpsDriver`（及 IMU serial）经 Stream 读数据，Parser 仍独立。

- [x] 定义 `Stream`（Connect/Disconnect/Read/Write/Status）与 `CreateSerialStream`
- [x] `SerialStream` 包装既有 `SerialPort`
- [x] 改造 `SerialGpsDriver` / `SerialImuDriver` 使用 `Stream`
- [x] 补充或调整单元测试（若有 stream 测；至少保证 nmea/wit 仍编过）

---

### Task 3: Lidar 基类骨架（Scan → PointCloud + SourceType）

**Deliverable:** `lidar/source_type.hpp` + `lidar_component_base.hpp`；`Lidar2d/3dModule` 文档注释指向该基类；暂不接真实 UDP 厂商。

- [x] `SourceType { kOnline, kRawPacket }`
- [x] `LidarComponentBase` 接口：`InitBase` / `WriteScan` / `WritePointCloud` 钩子（与 SensorPlugin 协作的设计注释）
- [x] 在 docs `backends.md` / `api/overview.md` 写明扩展路径

---

### Task 4: 诊断状态（薄层）

**Deliverable:** `common/status.hpp` + Stream/Manager 可上报的枚举；可选在 Stream 断连时更新状态字段。

- [x] `DeviceStatus { kOk, kDisconnected, kError }` + 简单 `DiagnosticSnapshot`
- [x] Stream Status 与 Diagnostic 对齐
- [x] FAQ/backends 一句说明

---

### Task 5: Proto schema 演进（不强制立刻替换 YAML）

**Deliverable:** 扩展 `autodriver_conf.proto`：`LidarConfigBase`、`SourceType`、域配置 oneof 草案；文档标明「运行时仍 YAML，proto 为契约」。

- [x] 扩展 proto 字段与注释
- [x] configuration.md 增加「与 Apollo 对齐的配置演进」小节

---

### Task 6: 文档同步

**Deliverable:** `docs/source` 反映新目录与扩展方式。

- [x] index / backends / api / faq 更新路径与 Stream/Lidar 基类
- [x] mkdocs nav 如需加「架构演进」可并入 api

---

## 后续计划

- [x] 首个真 lidar（Velodyne）UDP + Convert（VLP-16 几何）
- [x] UDP Stream + ReconnectStream
- [x] CAN `ProtocolData` / `MessageManager` 骨架
- [x] Proto 运行时加载（`.pb.txt` / `.pb` → `ConfigFromProto`）
- [x] 诊断发布（`ReportDiagnostic` → `/diagnostics` DiagnosticArray）
- [x] Compensator / 标定 YAML
- [x] Lidar 厂商静态注册表 + VelodyneLidarConf
- [x] Camera 按厂商分子目录 + Camera/PointCloud 注册表 + Orbbec 骨架
- [x] P0：Lidar 三段接口 / PoseBuffer / TCP+NTRIP / Velodyne 标定 / Hesai 骨架 / CanReceiver
- [x] Hesai / 其他厂商 Convert（真几何）— 仅 PandarXT / XT32；XT32M2X 等另开
- [x] Orbbec SDK 真采集（无 SDK 时 Create→nullptr；有 SDK 时 hub+camera+pointcloud）
- [x] CAN IMU/GPS 迁到 CanReceiver
