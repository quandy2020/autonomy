(bridge-rpc-guide)=
# 0. Bridge RPC 指南

`autonomy/bridge/proto` 定义 Bridge 对外 **gRPC 契约**与 **Lua 配置** 的 Protobuf Schema。本文说明文件划分、生成物与调用方式；逐 RPC 字段见 [§1 AutonomyService](01_autonomy_service.md)，配置消息见 [§2 BridgeOptions](02_bridge_options.md)。

| 本文 §0 | 相关文档 |
|---------|----------|
| Proto 索引、调用前提 | [§0 Bridge 指南](../00_guide.md) · [§3 gRPC](../03_grpc.md) · [§2 架构](../02_architecture.md) |

---

## 0.1 Proto 文件

| 文件 | 包名 | 用途 |
|------|------|------|
| `external_command_service.proto` | `autonomy.bridge.proto` | `AutonomyService` 及导航/探索指令消息 |
| `bridge_options.proto` | `autonomy.bridge.proto` | `BridgeOptions` / `GrpcOptions` / `MqttOptions`（Lua → C++ 配置） |

依赖的 commmsgs 类型：

| 导入 | 用途 |
|------|------|
| `geometry_msgs.proto` | `PoseArray`、`Polygon` |
| `vehicle_msgs.proto` | `RobotState`、`RobotEvent`（流式上行，字段待补充） |
| `builtin_interfaces.proto` | `Time`（请求时间戳） |
| `google/protobuf/empty.proto` | 无参 Server Stream 入口 |

源码路径：`autonomy/bridge/proto/`。Bazel/CMake 构建后生成 `*.pb.h`、`*.grpc.pb.h` 与 Python `*_pb2.py` / `*_pb2_grpc.py`。

---

## 0.2 服务一览

**服务全名**：`autonomy.bridge.proto.AutonomyService`

| RPC | 类型 | Handler | 状态 |
|-----|------|---------|------|
| `SendNavigationCommand` | Unary → Server Stream | `SendNavigationHandler` | ⏳ 已注册，业务待接线 |
| `SendExplorationCommand` | Unary → Server Stream | `SendExplorationHandler` | ⏳ 已注册，业务待接线 |
| `ReceiveBotStates` | Unary → Server Stream | — | ❌ Proto 已定义，未注册 |
| `ReceiveBotEvents` | Unary → Server Stream | — | ❌ Proto 已定义，未注册 |

<div class="comm-flow-diagram">
<div class="comm-flow-header">指令类 RPC（导航 / 探索）</div>
<div class="comm-flow-pipeline">
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-label">Client</span>
    <span class="comm-flow-step-title">Unary Request</span>
    <span class="comm-flow-step-sub"><code>NavigationCommandRequest</code> 等</span>
  </div>
  <div class="comm-flow-link">
    <span class="comm-flow-link-label">Server Stream</span>
    <span class="comm-flow-link-arrow" aria-hidden="true">→</span>
  </div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-label">Bridge</span>
    <span class="comm-flow-step-title">多次 <code>Send()</code></span>
    <span class="comm-flow-step-sub">进度 / 终态 <code>*CommandResponse</code></span>
  </div>
</div>
</div>

<div class="comm-flow-diagram">
<div class="comm-flow-header">状态类 RPC（规划）</div>
<div class="comm-flow-pipeline">
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-label">Client</span>
    <span class="comm-flow-step-title"><code>Empty</code></span>
  </div>
  <div class="comm-flow-link">
    <span class="comm-flow-link-label">Server Stream</span>
    <span class="comm-flow-link-arrow" aria-hidden="true">→</span>
  </div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-label">Bridge</span>
    <span class="comm-flow-step-title"><code>RobotState</code> / <code>RobotEvent</code></span>
  </div>
</div>
</div>

---

## 0.3 调用前提

1. Bridge 进程已 `BridgeServer::Start()`，默认监听 `127.0.0.1:5005`（见 [§0.5 配置](../00_guide.md#05-配置与-api)）  
2. 客户端与机器人网络可达；生产环境应启用 TLS（`GrpcOptions.enable_ssl_encryption`，见 [§2](02_bridge_options.md)）  
3. 当前**未启用** gRPC reflection；`grpcurl` 需 `-proto` / `-import-path` 指向 proto 根目录  

**列出服务**（需提供描述文件时）：

```bash
grpcurl -plaintext \
  -import-path autonomy/bridge/proto \
  -import-path autonomy/commsgs/proto \
  -proto external_command_service.proto \
  127.0.0.1:5005 list autonomy.bridge.proto.AutonomyService
```

**探测 RPC**（示例：停止导航）：

```bash
grpcurl -plaintext -d '{"command": 2}' 127.0.0.1:5005 \
  autonomy.bridge.proto.AutonomyService/SendNavigationCommand
```

---

## 0.4 客户端生成

| 语言 | 依赖 | Stub |
|------|------|------|
| C++ | `grpc++`、`external_command_service.grpc.pb.h` | `AutonomyService::Stub` |
| Python | `grpcio`、生成 `external_command_service_pb2_grpc` | `AutonomyServiceStub` |
| 其他 | `protoc` + `grpc_*_plugin` | 与 [gRPC 官方](https://grpc.io/docs/languages/) 一致 |

Python 最小示例见 [§1.5 客户端示例](01_autonomy_service.md#15-客户端示例python)。

---

## 0.5 实现与文档索引

| 主题 | 文档 |
|------|------|
| 各 RPC 字段、枚举、示例 JSON | [§1 AutonomyService](01_autonomy_service.md) |
| `BridgeOptions` 与 `bridge.lua` | [§2 BridgeOptions](02_bridge_options.md) |
| Handler 注册、线程、状态码 | [§3 gRPC 插件](../03_grpc.md) |
| 协议选型 | [§6 综述](../06_survey.md) |

---

**导航**：[§1 AutonomyService →](01_autonomy_service.md)
