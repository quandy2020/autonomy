# Orbbec

深度相机（本仓库以 **Gemini 330** 为主）。经 OrbbecSDK 开流，多路共享
`camera/orbbec/device_hub`。

## 简介

- **消息**：`Image`、点云 `PointCloud2`；板载 IMU 走 `imu` + `backend: orbbec`
- **Backend**：`orbbec`
- **源码**：`autodriver/camera/orbbec/`
- **厂商参数**：`config/camera/orbbec/gemini_330.yaml`
- **折叠示例**：`config/examples/orbbec_gemini_330.yaml`

## 依赖

| 项 | 说明 |
|---|---|
| CMake | `AUTODRIVER_WITH_ORBBEC=ON` |
| 库 | OrbbecSDK（`find_package(OrbbecSDK)`） |
| 权限 | USB / 厂商 udev |

无 SDK 时 stub。

## 配置

推荐折叠（与 RealSense 同语法），或把示例拷入主配置：

```bash
# 参考完整 Gemini 330 折叠写法
cat config/examples/orbbec_gemini_330.yaml
```

要点：

| 字段 | 建议 |
|---|---|
| `width` / `height` / `fps` | `0` = SDK 默认（对齐 OrbbecSDK_ROS2） |
| `device_preset` | `Default` |
| `disparity_to_depth_mode` | `HW` |
| `enable_laser` | `true`（官方默认） |

通道对齐 OrbbecSDK_ROS2：`/camera/color|depth|left_ir|right_ir/image_raw`，
点云 `/camera/depth/points` 或 `/camera/depth_registered/points`。

## 使用要点

1. 与 RealSense **不要同时默认 enable** 同一组 `/camera/*` 通道，避免话题冲突。
2. 深度质量参数集中在 `gemini_330.yaml`，主文件只管 enable / channel / stream。
3. `camera_info`：hub 会填内参；Writer 策略以 Publisher 实现为准。

## 相关

- [配置 · camera](../guide/configuration.md#camera)
- [后端 · Orbbec](../guide/backends.md#orbbec)
