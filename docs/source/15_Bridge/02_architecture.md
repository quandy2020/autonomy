(bridge-architecture)=
# 2. Bridge 模块架构设计

`autonomy/bridge` 的分层、数据流与组件边界。上手见 [§0](00_guide.md)；RPC 字段见 [RPC 参考](rpcs/01_autonomy_service.md)；插件实现见 [§3](03_grpc.md)。

| 本文 §2 | 相关文档 |
|---------|----------|
| 分层、流程、实现状态 | [§0 指南](00_guide.md) · [§3 gRPC](03_grpc.md) · [RPC](rpcs/00_guide.md) |

---

## 2.1 设计目标

1. **协议与业务解耦** — 传输层（`GrpcBridgeServer` / `MqttBridge`）与指令语义（导航 / 探索 FSM）分离，便于新增传输插件  
2. **配置驱动** — `config/bridge/bridge.lua` → `proto::BridgeOptions`，与 planning / control 模块一致  
3. **异步非阻塞** — 基于 `autonomy::common::async_grpc` Completion Queue，避免阻塞导航主线程  
4. **流式反馈** — `AutonomyService` 指令类 RPC 采用 Server Streaming 推送执行进度  
5. **安全可扩展** — `GrpcOptions` 预留 `enable_ssl_encryption`、`enable_google_auth`、`uplink_server_address`  

---

## 2.2 分层架构

Bridge 是 Autonomy 的**场外边界模块**：场外进程通过 `autonomy.bridge.proto.AutonomyService`（gRPC）或 MQTT Topic 接入；机载侧仅通过 `NavigatorStub` 等与 Navigator / Exploration 交互，**不**直接暴露 `autolink` Channel。

| 边界 | 说明 |
|------|------|
| 上游（L1） | 移动端 App、云平台、调度系统；使用 Protobuf / JSON 载荷 |
| 下游（L4） | [Navigator](../16_Navigator/index.rst)、Exploration、Planning、Control |
| 包范围 | `autonomy/bridge/**` 含服务入口、插件、Handler、配置；**不含** L1 客户端与 L4 算法实现 |
| 消息契约 | 对外 `external_command_service.proto`；对内 `commsgs` / Navigator Action |
| 上行 | `Stream` 推送 `NavigationCommandResponse`、`ExplorationCommandResponse`；规划 `vehicle_msgs.RobotState` / `RobotEvent` |

### 2.2.1 纵向分层

自上而下为**调用方向**（场外 → 机载）；最底管道标注**状态 Stream 回传**路径。

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">L1 场外</span>
      <span class="plan-arch-title">移动端 · 云平台 · 调度系统</span>
      <span class="plan-arch-sub">调用方，不属于 <code>autonomy/bridge</code> 包</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-body-block">
        <div class="nav-body-label">接入方式</div>
        <div class="nav-chip-list">
          <span class="nav-chip">gRPC + Protobuf</span>
          <span class="nav-chip">MQTT Pub/Sub</span>
          <span class="nav-chip">grpcurl / SDK</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>AutonomyService RPC / MQTT Topic</span></div>

  <div class="plan-arch-layer plan-arch-server">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">L2 服务</span>
      <span class="plan-arch-title">BridgeServer</span>
      <span class="plan-arch-sub"><code>bridge_server.hpp</code> · 模块唯一 C++ 入口</span>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">职责</div>
        <ul>
          <li>持有 <code>proto::BridgeOptions</code></li>
          <li>按 <code>use_grpc</code> / <code>use_mqtt</code> 构造插件</li>
          <li><code>Start()</code> / <code>WaitForShutdown()</code></li>
        </ul>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">配置</div>
        <div class="nav-chip-list">
          <span class="nav-chip">bridge.lua</span>
          <span class="nav-chip">LoadOptions</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>插件调度</span></div>

  <div class="plan-arch-layer plan-arch-plugin">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">L3 传输</span>
      <span class="plan-arch-title">GrpcBridgeServer · MqttBridge</span>
      <span class="plan-arch-sub"><code>plugins/grpc</code> · <code>plugins/mqtt</code>（规划中）</span>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">gRPC（已实现）</div>
        <div class="nav-chip-list">
          <span class="nav-chip">async_grpc::Server</span>
          <span class="nav-chip">RegisterHandler</span>
          <span class="nav-chip">CQ 线程池</span>
        </div>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">MQTT（空壳）</div>
        <div class="nav-chip-list">
          <span class="nav-chip">MqttBridge</span>
          <span class="nav-chip">Topic 映射</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>RpcHandler::OnRequest</span></div>

  <div class="plan-arch-layer plan-arch-map">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">L3 处理</span>
      <span class="plan-arch-title">Handler + Context</span>
      <span class="plan-arch-sub">协议载荷 → FSM → 机载调用</span>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">已注册 Handler</div>
        <div class="nav-chip-list">
          <span class="nav-chip">SendNavigationHandler</span>
          <span class="nav-chip">SendExplorationHandler</span>
        </div>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">上下文 / 客户端</div>
        <div class="nav-chip-list">
          <span class="nav-chip">GrpcBridgeContextInterface</span>
          <span class="nav-chip">NavigatorStub</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>FSM · TF · Navigator Action</span></div>

  <div class="plan-arch-layer plan-arch-post">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">L4 机载栈</span>
      <span class="plan-arch-title">Navigator · Exploration</span>
      <span class="plan-arch-sub">BT · Planning · Control · <code>vehicle_msgs</code></span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-chip-list">
        <span class="nav-chip">BtNavigator</span>
        <span class="nav-chip">PlannerServer</span>
        <span class="nav-chip">ControllerServer</span>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe plan-arch-pipe-out"><span>Server Stream 上行 · Response / RobotState / RobotEvent</span></div>

</div>

### 2.2.2 层职责对照

| 层 | 关键类型 | 输入 | 输出 | 实现状态 |
|----|----------|------|------|----------|
| L1 | — | 用户指令、目标位姿 | Stream 进度与状态 | 场外实现 |
| L2 | `BridgeServer` | `BridgeOptions` | 插件生命周期 | ✅ |
| L3 传输 | `GrpcBridgeServer` | HTTP/2 帧 | 调度至 Handler | ✅ 骨架 |
| L3 传输 | `MqttBridge` | Broker 消息 | 同左（规划） | ❌ 空壳 |
| L3 处理 | `SendNavigationHandler` | `NavigationCommandRequest` | `Stream<NavigationCommandResponse>` | ⏳ 待接线 |
| L3 处理 | `SendExplorationHandler` | `ExplorationCommandRequest` | `Stream<ExplorationCommandResponse>` | ⏳ 待接线 |
| L3 上下文 | `GrpcBridgeContextInterface` | Handler 内 `GetContext()` | Navigator / TF 依赖注入 | ⏳ 无业务字段 |
| L3 客户端 | `NavigatorStub` | Handler 调用 | 机载 Action 请求 | ❌ 空壳 |
| L4 | Navigator / Exploration | Action 目标 | 执行状态、位姿 | 机载栈 |

### 2.2.3 L3 传输与处理协作

`GrpcBridgeServer` 在构造阶段完成 Handler 注册：

```cpp
server_builder.RegisterHandler<handlers::SendNavigationHandler>();
server_builder.RegisterHandler<handlers::SendExplorationHandler>();
grpc_server_ = server_builder.Build();
```

事件到达后，`async_grpc` 在 Event 线程池调用 `SendNavigationHandler::OnRequest`；Handler 通过 `GrpcBridgeContextInterface`（继承 `ExecutionContext`）访问共享依赖，并调用 `NavigatorStub` 下发机载任务。MQTT 插件规划为**复用同一套 Handler 与 FSM**，仅替换 L3 传输入口（见 [§4 MQTT](04_mqtt.md)）。

| RPC（`AutonomyService`） | Handler 类 | 流模式 |
|--------------------------|-----------|--------|
| `SendNavigationCommand` | `SendNavigationHandler` | Unary → Server Stream |
| `SendExplorationCommand` | `SendExplorationHandler` | Unary → Server Stream |
| `ReceiveBotStates` | （未注册） | Empty → Server Stream |
| `ReceiveBotEvents` | （未注册） | Empty → Server Stream |

字段与 `grpcurl` 示例见 [RPC · AutonomyService](rpcs/01_autonomy_service.md)。

### 2.2.4 源码与依赖路径

| 组件 | 路径 | 类 / 接口 |
|------|------|-----------|
| 服务入口 | `bridge/bridge_server.hpp` | `BridgeServer` |
| gRPC 插件 | `bridge/plugins/grpc/grpc_bridge.hpp` | `GrpcBridgeServer` |
| 插件接口 | `bridge/plugins/grpc/grpc_bridge_server_interface.hpp` | `GrpcBridgeServerInterface` |
| 导航 Handler | `bridge/plugins/grpc/handlers/navigation_handler.hpp` | `SendNavigationHandler` |
| 探索 Handler | `bridge/plugins/grpc/handlers/exploration_handler.hpp` | `SendExplorationHandler` |
| 执行上下文 | `bridge/plugins/grpc/grpc_bridge_context.hpp` | `GrpcBridgeContextInterface` |
| Navigator 客户端 | `bridge/plugins/grpc/clients/navigator_stub.hpp` | `NavigatorStub` |
| 配置 | `bridge/common/bridge_option.hpp` | `LoadOptions` |
| Proto 服务 | `bridge/proto/external_command_service.proto` | `AutonomyService` |
| Proto 配置 | `bridge/proto/bridge_options.proto` | `BridgeOptions` |

---

## 2.3 配置加载

Bridge 与 planning / control 一致：**Lua 表 → `LuaParameterDictionary` → Protobuf**。配置不经过 gRPC 对外暴露，仅在进程内由 `BridgeServer` 消费。

| 入口 | 说明 |
|------|------|
| `config/bridge/bridge.lua` | 根表 `AUTONOMY_BRIDGE` |
| `config/autonomy.lua` | `include` bridge（根表挂载待完成） |
| `bridge_options.proto` | `BridgeOptions` / `GrpcOptions` / `MqttOptions` Schema |
| `common::LoadOptions` | `bridge/common/bridge_interface.cpp` |

### 2.3.1 纵向配置管线

自上而下为**配置从文件到运行时的加载顺序**。

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">文件</span>
      <span class="plan-arch-title">config/bridge/bridge.lua</span>
      <span class="plan-arch-sub">表名 <code>AUTONOMY_BRIDGE</code></span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-chip-list">
        <span class="nav-chip">use_grpc</span>
        <span class="nav-chip">use_mqtt</span>
        <span class="nav-chip">grpc { }</span>
        <span class="nav-chip">mqtt { }</span>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>ConfigurationFileResolver / include</span></div>

  <div class="plan-arch-layer plan-arch-server">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">解析</span>
      <span class="plan-arch-title">LuaParameterDictionary</span>
      <span class="plan-arch-sub"><code>autonomy::common</code></span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-body-block">
        <div class="nav-body-label">子字典</div>
        <div class="nav-chip-list">
          <span class="nav-chip">GetDictionary("grpc")</span>
          <span class="nav-chip">GetDictionary("mqtt")</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>common::LoadOptions</span></div>

  <div class="plan-arch-layer plan-arch-plugin">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">转换</span>
      <span class="plan-arch-title">CreateGrpcOptions · CreateMqttOptions</span>
      <span class="plan-arch-sub"><code>bridge/common/bridge_option.cpp</code></span>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">GrpcOptions</div>
        <ul>
          <li><code>host</code> · <code>port</code></li>
          <li><code>num_grpc_threads</code></li>
          <li><code>num_event_threads</code></li>
        </ul>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">MqttOptions</div>
        <ul>
          <li><code>host</code> · <code>port</code></li>
        </ul>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>mutable_grpc() / mutable_mqtt()</span></div>

  <div class="plan-arch-layer plan-arch-map">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">内存</span>
      <span class="plan-arch-title">proto::BridgeOptions</span>
      <span class="plan-arch-sub"><code>bridge_options.pb.h</code></span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-chip-list">
        <span class="nav-chip">use_grpc</span>
        <span class="nav-chip">use_mqtt</span>
        <span class="nav-chip">grpc</span>
        <span class="nav-chip">mqtt</span>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>BridgeServer(options)</span></div>

  <div class="plan-arch-layer plan-arch-post">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">运行时</span>
      <span class="plan-arch-title">BridgeServer · GrpcBridgeServer</span>
      <span class="plan-arch-sub">按标志位构造插件</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-body-block">
        <div class="nav-body-label">分支</div>
        <ul>
          <li><code>use_grpc</code> → <code>GrpcBridgeServer</code></li>
          <li><code>use_mqtt</code> → <code>MqttBridge</code>（待实现）</li>
        </ul>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe plan-arch-pipe-out"><span>待接线：GrpcOptions → Server::Builder</span></div>

</div>

### 2.3.2 LoadOptions 实现要点

```cpp
proto::BridgeOptions LoadOptions(LuaParameterDictionary* dict) {
    proto::BridgeOptions options;
    options.set_use_grpc(dict->GetBool("use_grpc"));
    options.set_use_mqtt(dict->GetBool("use_mqtt"));
    if (dict->HasKey("grpc")) {
        *options.mutable_grpc() =
            CreateGrpcOptions(dict->GetDictionary("grpc").get());
    }
    if (dict->HasKey("mqtt")) {
        *options.mutable_mqtt() =
            CreateMqttOptions(dict->GetDictionary("mqtt").get());
    }
    return options;
}
```

### 2.3.3 字段与 Server::Builder 对照

| `GrpcOptions` 字段 | `Server::Builder` 方法 | 当前状态 |
|---------------------|------------------------|----------|
| `host` + `port` | `SetServerAddress` | 硬编码 `127.0.0.1` |
| `num_grpc_threads` | `SetNumGrpcThreads` | 硬编码 `4` |
| `num_event_threads` | `SetNumEventThreads` | 硬编码 `4` |
| `enable_ssl_encryption` | TLS 凭据（待封装） | 未接线 |
| `uplink_server_address` | 上行 `NavigatorStub` | 预留 |

完整字段说明见 [RPC · BridgeOptions](rpcs/02_bridge_options.md) 与 [§0.5 配置](00_guide.md#05-配置与-api)。

---

## 2.4 进程生命周期

| 阶段 | 组件 / API | 行为 |
|------|------------|------|
| 构造 | `GrpcBridgeServer` | `RegisterHandler<SendNavigationHandler>`、`RegisterHandler<SendExplorationHandler>`，`Build()` |
| `BridgeServer::Start()` | `GrpcBridgeServer::Start()` | `grpc_server_->Start()`；启动 `task_thread_`（`ProcessSensorDataQueue` 占位） |
| 运行 | `async_grpc::Server` | gRPC 线程池处理 I/O；Event 线程池调度 `RpcHandler::OnRequest` |
| `Shutdown()` | `GrpcBridgeServer::Shutdown()` | 置 `shutting_down_`；`grpc_server_->Shutdown()`；join `task_thread_` |
| 阻塞退出 | `WaitForShutdown()` | 委托至 `grpc_server_->WaitForShutdown()` |

线程模型与 Builder 参数见 [§3.2](03_grpc.md#32-服务器构建)。

---

## 2.5 运行时数据流

单周期交互分两类：**指令类 RPC**（Unary 请求 + Server Stream 响应）与**状态类 RPC**（Empty 请求 + 持续 Stream，规划）。以下按**纵向调用顺序**描述；箭头自上而下为下行，最底管道标注上行 Stream。

### 2.5.1 SendNavigationCommand

| 项目 | 说明 |
|------|------|
| 服务 | `autonomy.bridge.proto.AutonomyService` |
| 方法 | `SendNavigationCommand` |
| Handler | `SendNavigationHandler` |
| 请求 | `NavigationCommandRequest`（`timestamp` · `NavigationCommand` · `PoseArray`） |
| 响应 | `Stream<NavigationCommandResponse>` |

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">① 场外</span>
      <span class="plan-arch-title">gRPC Client</span>
      <span class="plan-arch-sub">Unary <code>NavigationCommandRequest</code></span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-chip-list">
        <span class="nav-chip">NAV_CMD_START</span>
        <span class="nav-chip">geometry_msgs.PoseArray</span>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>HTTP/2 · /AutonomyService/SendNavigationCommand</span></div>

  <div class="plan-arch-layer plan-arch-server">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">② 传输</span>
      <span class="plan-arch-title">GrpcBridgeServer</span>
      <span class="plan-arch-sub"><code>async_grpc::Server</code> · CQ 分发</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-chip-list">
        <span class="nav-chip">gRPC 线程池</span>
        <span class="nav-chip">Event 线程池</span>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>RpcHandler::OnRequest</span></div>

  <div class="plan-arch-layer plan-arch-plugin">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">③ 处理</span>
      <span class="plan-arch-title">SendNavigationHandler</span>
      <span class="plan-arch-sub"><code>navigation_handler.cpp</code></span>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">校验</div>
        <ul>
          <li>导航 FSM 合法性</li>
          <li><code>timestamp</code> 窗口（规划）</li>
        </ul>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">变换</div>
        <ul>
          <li><code>PoseArray</code> → 全局系 TF</li>
          <li><code>GrpcBridgeContextInterface</code></li>
        </ul>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>NavigatorStub::SendGoal（待实现）</span></div>

  <div class="plan-arch-layer plan-arch-map">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">④ 机载</span>
      <span class="plan-arch-title">Navigator · BtNavigator</span>
      <span class="plan-arch-sub">NavigateToPose / 多点巡航 Action</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-chip-list">
        <span class="nav-chip">Planning</span>
        <span class="nav-chip">Control</span>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe plan-arch-pipe-out"><span>Server Stream · NavigationCommandResponse · Send() 多次 · Finish(Status)</span></div>

</div>

| 阶段 | API / 消息 | 说明 |
|------|------------|------|
| ① | `NavigationCommandRequest` | `NAV_CMD_START` 须带 `poses`；见 [RPC §1.2](rpcs/01_autonomy_service.md#12-sendnavigationcommand) |
| ② | `GrpcBridgeServer` | 根据路径路由至 `SendNavigationHandler` |
| ③ | `OnRequest` | 当前为空实现；应 `Send()` 接受确认再调 Navigator |
| ④ | Navigator Action | `cancel_goal` / 暂停 / 恢复映射见 [RPC §1.2](rpcs/01_autonomy_service.md#12-sendnavigationcommand) |
| 上行 | `NavigationCommandResponse` | `success` + `message`；RPC 存活期间可多次推送 |

### 2.5.2 SendExplorationCommand

与 §2.5.1 **同型**：`ExplorationCommandRequest` → `GrpcBridgeServer` → `SendExplorationHandler` → Exploration 模块 → `Stream<ExplorationCommandResponse>`（含 `ExplorationStatus`）。

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">① 场外</span>
      <span class="plan-arch-title">ExplorationCommandRequest</span>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>/AutonomyService/SendExplorationCommand</span></div>

  <div class="plan-arch-layer plan-arch-plugin">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">②③</span>
      <span class="plan-arch-title">GrpcBridgeServer → SendExplorationHandler</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-chip-list">
        <span class="nav-chip">oneof area</span>
        <span class="nav-chip">oneof map_name</span>
        <span class="nav-chip">EXPLORATION_CMD_*</span>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>Exploration 模块</span></div>

  <div class="plan-arch-layer plan-arch-post">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">④</span>
      <span class="plan-arch-title">Exploration</span>
    </div>
  </div>

  <div class="plan-arch-pipe plan-arch-pipe-out"><span>Stream · ExplorationCommandResponse</span></div>

</div>

字段与 `grpcurl` 见 [RPC §1.3](rpcs/01_autonomy_service.md#13-sendexplorationcommand)。

### 2.5.3 ReceiveBotStates / ReceiveBotEvents（规划）

状态类 RPC：**无请求体**，建立长连接后持续 Server Stream。Handler 尚未注册；`vehicle_msgs.RobotState` / `RobotEvent` 字段待补。

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-map">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">④ 机载</span>
      <span class="plan-arch-title">vehicle_msgs 状态源</span>
      <span class="plan-arch-sub">RobotState · RobotEvent</span>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>Bridge 内部订阅 / 轮询（待实现）</span></div>

  <div class="plan-arch-layer plan-arch-plugin">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">③</span>
      <span class="plan-arch-title">ReceiveBotStates Handler</span>
      <span class="plan-arch-sub">未注册</span>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>GrpcBridgeServer Stream</span></div>

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">① 场外</span>
      <span class="plan-arch-title">gRPC Client</span>
      <span class="plan-arch-sub">循环 Read() 直至 Cancel</span>
    </div>
  </div>

</div>

详见 [RPC §1.4](rpcs/01_autonomy_service.md#14-receivebotstates-receivebotevents)。

### 2.5.4 RPC 与 Handler 对照

| RPC 路径 | Handler 类 | 流模式 | 状态 |
|----------|-----------|--------|------|
| `/AutonomyService/SendNavigationCommand` | `SendNavigationHandler` | Unary → Server Stream | ⏳ 已注册，待接线 |
| `/AutonomyService/SendExplorationCommand` | `SendExplorationHandler` | Unary → Server Stream | ⏳ 已注册，待接线 |
| `/AutonomyService/ReceiveBotStates` | — | Empty → Server Stream | ❌ 未注册 |
| `/AutonomyService/ReceiveBotEvents` | — | Empty → Server Stream | ❌ 未注册 |

---

## 2.6 实现状态

| 组件 | 状态 | 说明 |
|------|------|------|
| `BridgeServer` 调度 | ✅ | `Start` / `WaitForShutdown` |
| `GrpcBridgeServer` | ✅ | `Server::Builder`、Handler 注册、后台线程 |
| `SendNavigationHandler` / `SendExplorationHandler` | ⏳ | 已注册，`OnRequest` 为空 |
| `ReceiveBotStates` / `ReceiveBotEvents` | ❌ | Proto 已定义，Handler 未实现 |
| `MqttBridge` | ❌ | 空壳 |
| `GrpcBridgeContextInterface` | ⏳ | 继承 `ExecutionContext`，无业务字段 |
| `NavigatorStub` | ❌ | 空壳 |
| Lua → `Server::Builder` | ⏳ | `BridgeOptions` 已解析，未传入 `GrpcBridgeServer` |

---

## 2.7 扩展与依赖

| 扩展点 | 方式 |
|--------|------|
| 新 RPC | 修改 `external_command_service.proto` + 新增 `RpcHandler` + `RegisterHandler`（[§3.5](03_grpc.md#35-自定义-handler)） |
| 新传输协议 | 实现 `BridgeInterface` 子类，由 `BridgeServer` 调度 |
| 认证中间件 | `async_grpc::Server::Builder` 拦截器（待封装） |
| 上行云端 | `GrpcOptions.uplink_server_address` + `NavigatorStub` |

**包内依赖**：

```
bridge_server (BridgeServer)
    └── plugins/grpc/grpc_bridge (GrpcBridgeServer)
            ├── async_grpc::Server
            ├── handlers::SendNavigationHandler
            ├── handlers::SendExplorationHandler
            └── grpc_bridge_context (GrpcBridgeContextInterface)
                    └── clients/navigator_stub (NavigatorStub)
```

| 外部依赖 | 用途 |
|----------|------|
| `grpc++` | gRPC C++ 运行时 |
| `protobuf` | `external_command_service.proto` / `bridge_options.proto` 序列化 |
| `autonomy/common/async_grpc` | 异步 gRPC 服务器框架 |
| `autonomy/commsgs/proto` | `geometry_msgs`、`vehicle_msgs`、`builtin_interfaces` |

---

**导航**：[← §1 概览](01_overview.md) · [§3 gRPC →](03_grpc.md)
