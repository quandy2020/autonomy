# autodriver 骨架扩展 Design（方案 A）

**Date:** 2026-09-05  
**Status:** Approved (chat)  
**Scope:** canbus 增强 + radar/microphone/smartereye 骨架 + GNSS parser 工厂 + Lidar PacketQueue

## Goals

1. `canbus/`：迁入 ProtocolData/Receiver；补 Client（Socket+Fake）、Sender、`byte.hpp`
2. `radar/` / `microphone/` / `smartereye/`：Registry + Module；无 SDK/真协议时 Create→nullptr
3. GNSS：`gps/parser` 工厂骨架（NMEA 注册）；不改 Cyber
4. Lidar：`packet_queue.hpp` 有界队列
5. 文档与单测；不主动 commit

## Non-goals

Cyber DAG；Conti 全协议表；PortAudio/Smartereye SDK 真采集；毫米波真机联调

## Layout

```
canbus/{byte,protocol_data,can_receiver,can_sender,can_client,README}
radar/{backend_registry,backend_register,conti/driver stub}
microphone/{backend_registry,respeaker stub}
smartereye/{camera_driver stub}  # CameraBackendRegistry
gps/parser/{parser.hpp, nmea_register}
lidar/packet_queue.hpp
```

`common/can_*.hpp` 保留转发 include，避免大破。
