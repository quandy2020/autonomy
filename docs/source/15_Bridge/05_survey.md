(bridge-survey)=
# 5. 协议综述

实现 [02](02_architecture.md)，契约 [03](03_rpc_protocol.md)。

在 rosbridge / Foxglove / REST 与本栈 gRPC 之间做选型。

Bridge 到 Stub/GoalChannel；BT 在 TaskServer（autonomy/task/）。

### 现在有什么

- **gRPC + `automsgs.rpcs.*`**：主路径  
- **async_grpc 服务端**：框架在；各域 Handler 接线进度不一，见 [grpc/07](grpc/07_handlers.md)  
- **Unary**：Get* / Estop / CancelAll  
- **Server Streaming**：Navigate / Explore / Follow / Charge 等  
- **Bidi**：Teleop  

TLS / 认证 / 云上 uplink 在 Proto 里有预留，产品化未闭合。BotStates / BotEvents Stream 仍在演进。

### 和常见方案的差别

| 方案 | 传输 | 备注 |
|------|------|------|
| rosbridge | WS + JSON | 弱类型，偏临时调试 |
| foxglove_bridge | WS + Protobuf | 可视化，不是命令面 |
| Apollo Gateway | gRPC / HTTP | 与 async_grpc 思路近 |
| Autoware AD API | REST / WS | 绑 ROS 2 |
| Nav2 commander | 进程内 | 无跨网契约 |

Autonomy 对外命令面以 **gRPC + Protobuf** 为准。

### 实用选型

| 场景 | 建议 |
|------|------|
| 本机调试 | `127.0.0.1:5005`，先 `GetCapabilities` |
| 局域网 App | gRPC；量产再上 TLS |
| 云端调度 | 优先 Stream；注意断线重连与取消 |

联调顺序见 [00 指南](00_guide.md) 与 [rpcs/01](rpcs/01_connection_guide.md)。
