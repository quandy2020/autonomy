(bridge-guide)=
# 0. Bridge 通信桥接指南

`autonomy/bridge` 在**机载导航栈**与**场外客户端**（移动端、云平台、调度系统）之间提供 gRPC / MQTT 桥接，对标 `rosbridge_suite`、Nav2 gRPC 封装。

| 本文 §0 | 相关文档 |
|---------|----------|
| 上手、配置、排错 | [§2 架构](02_architecture.md) · [§6 综述](06_survey.md) |

<div class="nav-costmap-banner">
  <strong>Bridge 在 Autonomy 栈中的位置</strong>
  <span class="nav-costmap-detail">场外 gRPC / MQTT → BridgeServer → Navigator / Exploration</span>
  <span class="nav-costmap-arrow">分层与数据流见 §2 →</span>
</div>

---

## 0.1 文档地图

### 按角色阅读

| 角色 | 阅读顺序 |
|------|----------|
| 新手 | §0.2 → [§2 架构](02_architecture.md) → §0.5 |
| 集成调试 | §0.2、§0.5 → [RPC · AutonomyService](rpcs/01_autonomy_service.md) → [§0.8 排错](#08-排错) |
| 协议研发 | [§6 综述](06_survey.md) → [RPC 指南](rpcs/00_guide.md) → [§3 gRPC](03_grpc.md) / [§4 MQTT](04_mqtt.md) |

### 章节目录

| § | 文档 | 内容 |
|---|------|------|
| 0 | 本指南 | 快速开始、配置、API、排错 |
| 1 | [概览](01_overview.md) | 定位、能力、相关模块 |
| 2 | [架构](02_architecture.md) | 分层、数据流、组件、实现状态 |
| 3 | [gRPC](03_grpc.md) | `GrpcBridgeServer`、Handler、RPC |
| 4 | [MQTT](04_mqtt.md) | 规划中的 Pub/Sub 插件 |
| 5 | [RPCs 协议](rpcs/00_guide.md) | `AutonomyService` 字段与调用示例 |
| 6 | [综述](06_survey.md) | 协议选型、生态对比 |

---

## 0.2 快速开始

1. 编辑 `config/bridge/bridge.lua`：`use_grpc` / `use_mqtt`、监听地址  
2. C++ 加载 `LoadOptions` → 构造 `BridgeServer` → `Start()`  
3. 外部客户端连接 `host:port`（gRPC）或 Broker（MQTT，规划中）

```cpp
#include "autonomy/bridge/bridge_server.hpp"
#include "autonomy/bridge/common/bridge_interface.hpp"

auto options = autonomy::bridge::common::LoadOptions(bridge_dict);
auto server = std::make_shared<autonomy::bridge::BridgeServer>(options);
server->Start();
server->WaitForShutdown();
```

```lua
-- config/bridge/bridge.lua
AUTONOMY_BRIDGE = {
    use_grpc = true,
    use_mqtt = false,
    grpc = { host = "127.0.0.1", port = 5005, num_grpc_threads = 5 },
}
```

```bash
grpcurl -plaintext 127.0.0.1:5005 list
grpcurl -plaintext -d '{"command": 2}' 127.0.0.1:5005 \
  autonomy.bridge.proto.AutonomyService/SendNavigationCommand
```

> `GrpcBridgeServer` 当前部分参数仍硬编码，见 [§0.5 配置](#05-配置与-api)。`bridge_main.cpp` 为 stub，生产建议在 `system::Autonomy` 内嵌 `BridgeServer`。

联调：终端 1 启动导航栈，终端 2 启动 Bridge，客户端经 gRPC 下发目标。协作流程见 [§0.6](#06-安全与导航栈协作)。

---

## 0.3 配置入口

| 路径 | 说明 |
|------|------|
| `config/bridge/bridge.lua` | `AUTONOMY_BRIDGE` 表 |
| `config/autonomy.lua` | `include` bridge（根表挂载待完成） |
| `bridge_options.proto` | `BridgeOptions` / `GrpcOptions` |

C++：`common::LoadOptions(LuaParameterDictionary*)` → `proto::BridgeOptions`。字段见 [§0.5](#05-配置与-api)。

---

## 0.4 实现状态（摘要）

完整表格见 [§2.6 实现状态](02_architecture.md#26-实现状态)。

| 组件 | 状态 |
|------|------|
| `BridgeServer` / `GrpcBridgeServer` | ✅ 生命周期、Handler 注册 |
| Navigation / Exploration Handler | ⏳ 骨架已注册，业务待接线 |
| `ReceiveBotStates` / `Events` | ❌ Proto 已有，Handler 未实现 |
| `MqttBridge` / `NavigatorStub` | ❌ 空壳 |

---

## 0.5 配置与 API

(bridge-usage)=

**`config/bridge/bridge.lua`**

```lua
AUTONOMY_BRIDGE = {
    use_grpc = true,
    use_mqtt = false,
    grpc = { host, port, num_grpc_threads, num_event_threads },
    mqtt = { host, port },
}
```

| 字段 | 类型 | 说明 | 默认 |
|------|------|------|------|
| `use_grpc` | bool | 启用 gRPC 桥接 | `true` |
| `use_mqtt` | bool | 启用 MQTT 桥接 | `false` |
| `grpc.host` | string | 监听地址 | `127.0.0.1` |
| `grpc.port` | uint | 监听端口 | `5005` |
| `grpc.num_grpc_threads` | uint | gRPC 工作线程 | `5` |
| `grpc.num_event_threads` | uint | 事件队列线程 | `5` |
| `grpc.uplink_server_address` | string | 上行云端地址（预留） | — |
| `grpc.enable_ssl_encryption` | bool | TLS 加密（预留） | — |
| `mqtt.host` / `mqtt.port` | — | MQTT Broker 地址 | `127.0.0.1:12345` |

> **已知限制**：`GrpcBridgeServer` 构造函数尚未读取 `options_` 中的 `host`/`port`/线程数，当前硬编码 `127.0.0.1`、4 线程。

**`BridgeServer`**

| API | 用途 |
|-----|------|
| `BridgeServer()` | 默认构造，创建 `GrpcBridgeServer` |
| `BridgeServer(options)` | 带配置构造 |
| `Start()` | 按 `use_grpc`/`use_mqtt` 启动对应插件 |
| `WaitForShutdown()` | 阻塞等待 gRPC 服务器退出 |

**外部指令映射**

| gRPC 方法 | 内部目标 | 状态 |
|-----------|----------|------|
| `SendNavigationCommand` | Navigator `compute_path_to_pose` / BT | ⏳ Handler 待接线 |
| `SendExplorationCommand` | Exploration 模块 | ⏳ Handler 待接线 |
| `ReceiveBotStates` | `vehicle_msgs::RobotState` 流 | ❌ 未注册 |
| `ReceiveBotEvents` | `vehicle_msgs::RobotEvent` 流 | ❌ 未注册 |

| 值 | 枚举 | 含义 |
|----|------|------|
| 1 | `NAV_CMD_START` | 开始导航，需 `poses` |
| 2 | `NAV_CMD_STOP` | 停止 |
| 3 | `NAV_CMD_CANCEL` | 取消 |
| 4 | `NAV_CMD_RESUME` | 恢复 |
| 5 | `NAV_CMD_PAUSE` | 暂停 |

RPC 字段与 `grpcurl` 示例见 [RPC · AutonomyService](rpcs/01_autonomy_service.md)。

**Python 客户端示例**

```python
import grpc
from autonomy.bridge.proto import external_command_service_pb2 as pb
from autonomy.bridge.proto import external_command_service_pb2_grpc as pb_grpc

channel = grpc.insecure_channel("127.0.0.1:5005")
stub = pb_grpc.AutonomyServiceStub(channel)

req = pb.NavigationCommandRequest(command=pb.NAV_CMD_START)
for resp in stub.SendNavigationCommand(req):
    print(resp.success, resp.message)
```

---

## 0.6 安全与导航栈协作

| 场景 | 建议 |
|------|------|
| 生产部署 | 启用 `enable_ssl_encryption`，配置双向 TLS |
| 公网暴露 | 配合 `enable_google_auth` 或自定义 Token 中间件 |
| 本地调试 | `127.0.0.1` 绑定，禁止 `0.0.0.0` 无认证开放 |
| 指令重放 | 校验 `timestamp`，拒绝过期请求 |

<div class="comm-flow-diagram">
<div class="comm-flow-header">Bridge 在导航栈边界</div>
<div class="comm-flow-pipeline comm-flow-pipeline--chain">
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-title">移动端 / 云平台</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">BridgeServer</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-title">Navigator → Planning → Control</span>
  </div>
</div>
</div>

Bridge 应在 Navigator Action 接受/拒绝时通过 Stream 推送中间状态，而非仅在 RPC 结束时返回单次响应。

---

## 0.7 扩展自定义 Handler

1. 在 `external_command_service.proto` 中声明新 RPC  
2. 使用 `DEFINE_HANDLER_SIGNATURE` + 继承 `RpcHandler`  
3. 在 `GrpcBridgeServer` 构造函数中 `RegisterHandler<YourHandler>()`  
4. 在 Handler 内通过 `GrpcBridgeContextInterface` 访问 Navigator / 地图等依赖  

模板见 [§3.5 自定义 Handler](03_grpc.md#35-自定义-handler)。

---

## 0.8 排错

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 端口连接拒绝 | Bridge 未 `Start()` | 确认进程已启动并监听 |
| `grpcurl list` 失败 | 无 reflection | 提供 `.proto` 描述文件 |
| 指令无响应 | Handler 为 stub | 检查 `navigation_handler.cpp` 是否已接线 |
| 位姿偏移 | TF 未就绪 | 等待 `transform` 发布后再发 `NAV_CMD_START` |
| 消息过大 | 超过 100 MB | 减少 `PoseArray` 点数或拆分请求 |
| MQTT 无效果 | `use_mqtt=false` 且插件未实现 | 使用 gRPC 或等待 MQTT 插件 |

---

**导航**：[§1 概览 →](01_overview.md) · [§2 架构](02_architecture.md)
