# microphone

麦克风采集骨架（ReSpeaker 路径）。

| 路径 | 说明 |
|---|---|
| `backend_registry.*` | `backend` 名 → 工厂 |
| `backend_register.hpp` | `REGISTER_MICROPHONE_BACKEND` |
| `respeaker_driver.*` | Stub（`backend: respeaker`）；需 PortAudio / USB |

样本暂为 `MicrophoneSample`（用 `sensor_msgs/Image` 装 PCM 字节）。

YAML 组：`microphone` → `MicrophoneModule`。详见 [backends](../../docs/source/guide/backends.md)。
