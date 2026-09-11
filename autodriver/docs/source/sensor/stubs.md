# Stub 与占位

Registry 已完成占位注册；多数情况下 `Create` 返回 `nullptr`，或仅为 Attach（无采集）。落地步骤参见 [架构 · 加厂商](../guide/architecture.md)。术语参见 [术语](../guide/glossary.md)。

> 本页所列均为**待实现**或消息占位，不得当作已交付采集能力。

| YAML | Module | backend | 消息占位 | 待实现 |
|---|---|---|---|---|
| `radar` | `RadarModule` | `conti`（`continental`） | PointCloud2 | Conti + canbus |
| `microphone` | `MicrophoneModule` | `respeaker` | Image 承载 PCM | PortAudio |
| `camera` | `CameraModule` | `smartereye` | Image | 厂商 SDK；`config/camera/smartereye/` |
| `range` | `RangeModule` | — | Range | 真实采集循环 |
| lidar_3d | — | `rslidar`/`robosense`、`lslidar`、`seyond`、`vanjee` | — | `vendor_stubs.cpp`；参照 Velodyne/Livox |

相关测试：`test_skeleton_modules`、`test_canbus_skeleton`。
