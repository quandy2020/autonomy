# autodriver 骨架模块 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or executing-plans task-by-task.

**Goal:** 落地方案 A：canbus 增强 + 三模态骨架 + GNSS parser + PacketQueue。

**Architecture:** SensorManager + 静态 Registry；canbus 为共享 CAN 层；radar/mic/smartereye 先 stub。

**Tech Stack:** C++17, CMake, Autolink/automsgs

**Spec:** `docs/superpowers/specs/2026-09-05-autodriver-skeleton-modules-design.md`

## Global Constraints

- 中文回复；不主动 commit；不引入 Cyber DAG
- 无第三方库时 Create→nullptr

---

### Task 1: canbus

- [x] `canbus/byte.hpp`、`protocol_data.hpp`、`can_receiver.hpp`（迁/拷）
- [x] `can_client.hpp` Socket+Fake；`can_sender.hpp`
- [x] `common/can_*.hpp` 转发；更新 IMU/GPS include
- [x] 单测 Fake 收发

### Task 2: SensorType + 三骨架

- [x] `kRadar` / `kMicrophone`；traits 通道后缀
- [x] radar/microphone registries + Conti/ReSpeaker stub
- [x] smartereye camera stub 注册
- [x] modules.cpp 注册 Module

### Task 3: GNSS parser + PacketQueue

- [x] `gps/parser/parser.hpp` + NMEA 注册钩子
- [x] `lidar/packet_queue.hpp`

### Task 4: 文档/CMake

- [x] backends / configuration / api；CMake 源列表
