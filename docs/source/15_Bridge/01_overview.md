(bridge-overview)=
# 1. 模块概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 层级 | 外部接口层（External Bridge） |
| 输入 | 外部导航/探索指令、状态订阅请求 |
| 输出 | `NavigationCommandResponse`、`ExplorationCommandResponse`、机器人状态/事件流 |
| 下游 | Navigator、Exploration、Localization、Visualization |
| 对标 | ROS 2 `rosbridge_suite`、Nav2 `nav2_simple_commander` gRPC 封装、Apollo Cyber RT Bridge |

Bridge 模块不直接参与路径规划或运动控制，而是将**外部命令语义**翻译为 Autonomy 内部服务调用，并将**内部状态**以 Protobuf / MQTT 载荷形式对外发布。

### 1.2 核心能力

- **gRPC 服务**：基于 `async_grpc::Server`，暴露 `AutonomyService`（导航、探索、状态流）
- **MQTT 桥接**（规划中）：轻量级 Pub/Sub，适合 IoT 与弱网场景
- **配置驱动**：Lua → `BridgeOptions` Protobuf，与全栈配置风格一致
- **插件化传输**：`GrpcBridgeServer` / `MqttBridge` 作为独立插件，由 `BridgeServer` 统一调度
- **指令状态机**：导航与探索命令均定义有限状态自动机（FSM），见 [§3.4](03_math.md#34-指令状态机)

### 1.3 源码结构

```
autonomy/bridge/
├── bridge_server.*              # 服务入口，调度 gRPC / MQTT
├── bridge_main.cpp              # 独立进程入口（当前为 stub）
├── common/
│   ├── bridge_interface.*       # LoadOptions()：Lua → BridgeOptions
│   └── bridge_option.*          # GrpcOptions / MqttOptions 解析
├── proto/
│   ├── bridge_options.proto     # 桥接配置
│   └── external_command_service.proto  # AutonomyService RPC 定义
└── plugins/
    ├── grpc/
    │   ├── grpc_bridge.*        # gRPC 服务器实现
    │   ├── grpc_bridge_context.*# ExecutionContext（待扩展）
    │   ├── handlers/            # SendNavigation / SendExploration
    │   └── clients/             # NavigatorStub（待实现）
    └── mqtt/
        └── mqtt_bridge.*        # MQTT 插件（待实现）
```

### 1.4 实现状态

| 组件 | 状态 | 说明 |
|------|------|------|
| `BridgeServer` 生命周期 | ✅ | 构造、Start、WaitForShutdown |
| `GrpcBridgeServer` 启动 | ✅ | 注册 Handler、启动线程池 |
| `SendNavigationHandler` | ⏳ | Handler 骨架已注册，业务逻辑待接线 |
| `SendExplorationHandler` | ⏳ | 同上 |
| `ReceiveBotStates/Events` | ❌ | Proto 已定义，Handler 未注册 |
| `MqttBridge` | ❌ | 头/源文件为空壳 |
| `NavigatorStub` 客户端 | ❌ | 待连接 Navigator 服务 |
| `config/autonomy.lua` 集成 | ⏳ | `bridge.lua` 已 include，根表未挂载 |

### 1.5 相关模块

- `autonomy/navigator` — 导航行为树，接收 `NAV_CMD_START` 等指令的目标位姿
- `autonomy/commsgs` — `geometry_msgs`、`vehicle_msgs` 等跨模块消息定义
- `autonomy/common/async_grpc` — 异步 gRPC 服务器框架（源自 Cartographer）
- `config/bridge/bridge.lua` — Bridge 运行时配置
