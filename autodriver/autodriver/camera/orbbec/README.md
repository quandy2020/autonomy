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

Params (aligned with RealSense): `serial`, `index`, `model`, `stream`
(`color`|`depth`|`ir`), `width`, `height`, `fps`, `frame_id`.
