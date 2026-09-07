# autodriver Camera 厂商目录 + Backend Registry Design

**Date:** 2026-09-05  
**Status:** Approved (chat)  
**Scope:** 方案 B 目录 + 方案 2（迁 RealSense、Camera/PointCloud 注册表、Orbbec 骨架）

## 目标

1. `camera/realsense/` 收纳现有 RealSense 源；`camera/orbbec/` 占位骨架。
2. `CameraBackendRegistry` / `PointCloudBackendRegistry`（镜像 lidar），`CameraModule` / `PointCloudModule` 查表。
3. Orbbec：`REGISTER_*("orbbec")`，无 SDK 时 Create 返回 `nullptr` + 明确日志。
4. RealSense IMU：仅改 include 路径；仍由 `ImuModule` 的 `backend == "realsense"` 分支创建（本轮不上 IMU 注册表）。

## 非目标

- 真 Orbbec SDK / 采集
- IMU 厂商注册表
- 消息类型变更、流水线拆分
- 主动 git commit

## 目录

```
camera/
  backend_registry.{hpp,cpp}   # Camera + PointCloud 两个单例，或同文件两类
  backend_register.hpp
  realsense/
    device_hub.*, camera_driver.*, imu_driver.*,
    pointcloud_driver.*, camera_info.*
  orbbec/
    stub_camera_driver.*, stub_pointcloud_driver.*  # Create → nullptr
```

`bridge/realsense_channels.hpp` 保留原路径（话题命名辅助，非厂商驱动）。

旧路径 `camera/realsense_*.hpp` 删除；全库更新 `#include`。

## 注册

- RealSense camera / pointcloud：`REGISTER_CAMERA_BACKEND` / `REGISTER_POINTCLOUD_BACKEND`（或统一宏带 tag）。
- Orbbec 同名注册，工厂打 `AERROR` 并返回空。
- CMake：`AUTODRIVER_WITH_REALSENSE` 仍控制 RealSense 源；Orbbec stub **始终编译**（无第三方库）。

## 成功标准

- include 为 `autodriver/camera/realsense/...`
- YAML `backend: realsense` 行为不变
- `backend: orbbec` Attach 失败且日志可读
- 文档 backends 说明目录与扩展步骤
