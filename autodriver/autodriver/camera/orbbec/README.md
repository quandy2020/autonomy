# Orbbec (奥比中光) camera backends

`backend: orbbec` for `camera` / `point_cloud` modules.

Requires OrbbecSDK (v2) at build time (`AUTODRIVER_WITH_ORBBEC=ON` +
`find_package(OrbbecSDK)`). Without the SDK, factories return `nullptr`.

| File | Role |
|---|---|
| `device_hub.*` | Shared pipeline (color / depth / IR / point cloud) |
| `camera_driver.*` | `CameraFrame` + CameraInfo |
| `pointcloud_driver.*` | Depth (+ optional RGB) → `PointCloud2` |
| `camera_info.*` | Intrinsics → `sensor_msgs/CameraInfo` |

Params: `serial`, `index`, `model`, `stream`
(`color`|`depth`|`left_ir`|`right_ir`|`ir0`), `width`/`height`/`fps`
(`0` = SDK default / `OB_*_ANY`), `frame_id`, plus Gemini 330 official
knobs from OrbbecSDK_ROS2 `gemini_330_series.launch.py`:
`device_preset` (Default), `disparity_to_depth_mode` (HW), `enable_laser`,
`enable_disparity_to_depth`, soft/HW noise removal, spatial filters.

## Channels (OrbbecSDK_ROS2 / Gemini 330)

Use the same topic names as OrbbecSDK_ROS2 `gemini_330_series.launch.py`
with `camera_name:=camera`. Streams in `config/autodriver_hardware.yaml`;
device params in `config/camera/orbbec/gemini_330.yaml` via `params_file`.

| Stream | Autolink channel | camera_info (auto) |
|---|---|---|
| color | `/camera/color/image_raw` | `/camera/color/camera_info` |
| depth | `/camera/depth/image_raw` | `/camera/depth/camera_info` |
| left_ir | `/camera/left_ir/image_raw` | `/camera/left_ir/camera_info` |
| right_ir | `/camera/right_ir/image_raw` | `/camera/right_ir/camera_info` |
| depth points | `/camera/depth/points` | — |
| RGB points | `/camera/depth_registered/points` | — |
| IMU (ROS2 only) | `/camera/gyro_accel/sample` | — |

`stream: ir` aliases to `left_ir` (Gemini 330 stereo). Use `ir0` for
legacy single-IR devices (`OB_STREAM_IR`).
Not the same as realsense-ros (`image_rect_raw`, `infra1`,
`/camera/depth/color/points`).