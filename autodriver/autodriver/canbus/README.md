# canbus

CAN 协议栈，供 IMU/GPS CAN、未来 Conti radar 等复用。

| 文件 | 职责 |
|---|---|
| `protocol_data.hpp` | `ProtocolData` / `MessageManager`：按 CAN id 解码并回调 |
| `can_receiver.hpp` | SocketCAN 收线程 → MessageManager |
| `can_client.hpp` | `SocketCanClient` + `FakeCanClient`（channel 以 `fake` 开头） |
| `can_sender.hpp` | 周期发帧任务 |
| `byte.hpp` | 单字节位域打包/解包 |

优先 `#include "autodriver/canbus/..."`。

单测：`test/test_canbus_skeleton.cpp`（Byte、FakeCan、Sender）。API 见 [api/overview](../../docs/source/api/overview.md)。
