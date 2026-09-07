# autodriver Camera Vendor Layout Implementation Plan

> Implemented inline after design approval (2026-09-05).

**Goal:** `camera/realsense/` + `camera/orbbec/` stub + Camera/PointCloud registries.

**Spec:** `docs/superpowers/specs/2026-09-05-autodriver-camera-vendor-layout-design.md`

## Done

- [x] git mv RealSense → `camera/realsense/{device_hub,camera_*,imu_*,pointcloud_*}`
- [x] `CameraBackendRegistry` / `PointCloudBackendRegistry` + macros
- [x] RealSense self-register; Orbbec stub register
- [x] `CameraModule` / `PointCloudModule` 查表
- [x] docs + align plan checkbox
- [x] `test_camera_backend_registry`
