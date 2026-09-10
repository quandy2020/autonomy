# Stub 与占位模态

下列 backend 已在注册表中占位，`Create` 当前返回 `nullptr`（或仅 Attach），
便于先写 YAML / 联调上层。落地步骤见 [后端 · 扩展](../guide/backends.md)。

## Radar

| 项 | 值 |
|---|---|
| YAML | `radar` |
| Module | `RadarModule` |
| Backend | `conti`（alias `continental`） |
| 消息占位 | `PointCloud2`（`RadarSample`） |

待 Conti ProtocolData + SocketCAN 真采集。

## Microphone

| 项 | 值 |
|---|---|
| YAML | `microphone` |
| Backend | `respeaker` |
| 消息占位 | `Image` 装 PCM |

待 PortAudio / USB HID。

## SmarterEye

| 项 | 值 |
|---|---|
| YAML | `camera` + `backend: smartereye` |
| 参数 | `config/camera/smartereye/autodriver.yaml` |

待厂商 SDK。

## Range（超声波）

| 项 | 值 |
|---|---|
| YAML | `range` |
| 状态 | **attach-only**（无真驱动循环） |

## 其它 3D 激光 stub

`rslidar` / `robosense`、`lslidar`、`seyond`、`vanjee`：见
`autodriver/lidar/vendor_stubs.cpp`。实现时参考 Velodyne / Livox 目录布局。
