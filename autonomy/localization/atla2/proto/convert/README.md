# convert

C++ ↔ Proto 字段镜像编解码（不依赖 codegen 即可单测）：

- `sensor_packet_codec.*` — `SensorData` ↔ `SyncedSensorPacketDto` 往返
- 开启 `ATLA2_HAS_PROTO` 后可在此扩展真实 `.pb.h` 映射
