# Stub 与占位

Registry 已占位；`Create`→`nullptr` 或 attach-only。落地：仿 [架构·加厂商](../guide/architecture.md)。

| YAML | Module | backend | 消息占位 | 待补 |
|---|---|---|---|---|
| `radar` | `RadarModule` | `conti`（`continental`） | PointCloud2 | Conti + canbus |
| `microphone` | `MicrophoneModule` | `respeaker` | Image=PCM | PortAudio |
| `camera` | `CameraModule` | `smartereye` | Image | 厂商 SDK；`config/camera/smartereye/` |
| `range` | `RangeModule` | — | Range | 真采集循环 |
| lidar_3d | — | `rslidar`/`robosense`、`lslidar`、`seyond`、`vanjee` | — | `vendor_stubs.cpp`；仿 Velodyne/Livox |

测：`test_skeleton_modules`、`test_canbus_skeleton`。
