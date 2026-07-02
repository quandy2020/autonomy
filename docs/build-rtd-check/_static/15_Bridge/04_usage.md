(bridge-usage)=
# 4. 使用指南

### 4.1 配置

**入口文件**：`config/bridge/bridge.lua`（已被 `config/autonomy.lua` include）

```lua
AUTONOMY_BRIDGE = {
    use_grpc = true,
    use_mqtt = false,
    grpc = { host, port, num_grpc_threads, num_event_threads },
    mqtt = { host, port },
}
```

**关键字段**：

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

C++ 加载：

```cpp
auto options = autonomy::bridge::common::LoadOptions(bridge_dict);
```

> **已知限制**：`GrpcBridgeServer` 构造函数尚未读取 `options_` 中的 `host`/`port`/线程数，当前硬编码 `127.0.0.1`、4 线程。配置接线完成后将与此表一致。

### 4.2 BridgeServer API

| API | 用途 |
|-----|------|
| `BridgeServer()` | 默认构造，创建 `GrpcBridgeServer` |
| `BridgeServer(options)` | 带配置构造 |
| `Start()` | 按 `use_grpc`/`use_mqtt` 启动对应插件 |
| `WaitForShutdown()` | 阻塞等待 gRPC 服务器退出 |

```cpp
class BridgeServer {
public:
    explicit BridgeServer();
    explicit BridgeServer(const proto::BridgeOptions& options);
    void Start();
    void WaitForShutdown();
};
```

### 4.3 外部指令映射

| gRPC 方法 | 内部目标 | 状态 |
|-----------|----------|------|
| `SendNavigationCommand` | Navigator `compute_path_to_pose` / BT | ⏳ Handler 待接线 |
| `SendExplorationCommand` | Exploration 模块 | ⏳ Handler 待接线 |
| `ReceiveBotStates` | `vehicle_msgs::RobotState` 流 | ❌ 未注册 |
| `ReceiveBotEvents` | `vehicle_msgs::RobotEvent` 流 | ❌ 未注册 |

**导航命令枚举**（`external_command_service.proto`）：

| 值 | 枚举 | 含义 |
|----|------|------|
| 1 | `NAV_CMD_START` | 开始导航，需 `poses` |
| 2 | `NAV_CMD_STOP` | 停止 |
| 3 | `NAV_CMD_CANCEL` | 取消 |
| 4 | `NAV_CMD_RESUME` | 恢复 |
| 5 | `NAV_CMD_PAUSE` | 暂停 |

### 4.4 客户端集成示例（Python）

```python
import grpc
from autonomy.bridge.proto import external_command_service_pb2 as pb
from autonomy.bridge.proto import external_command_service_pb2_grpc as pb_grpc

channel = grpc.insecure_channel("127.0.0.1:5005")
stub = pb_grpc.AutonomyServiceStub(channel)

req = pb.NavigationCommandRequest(
    command=pb.NAV_CMD_START,
    # poses=...  # 填充 PoseArray
)
for resp in stub.SendNavigationCommand(req):
    print(resp.success, resp.message)
```

### 4.5 安全建议

| 场景 | 建议 |
|------|------|
| 生产部署 | 启用 `enable_ssl_encryption`，配置双向 TLS |
| 公网暴露 | 配合 `enable_google_auth` 或自定义 Token 中间件 |
| 本地调试 | `127.0.0.1` 绑定，禁止 `0.0.0.0` 无认证开放 |
| 指令重放 | 校验 `timestamp`，拒绝过期请求（见 [§3.7](03_math.md#37-时间同步)） |

### 4.6 与导航栈协作

```
移动端 / 云平台
      │ gRPC
      ▼
 BridgeServer ──► Navigator BT ──► Planning ──► Control
      ▲                │
      │    RobotState  │
      └────────────────┘
```

Bridge 应在 Navigator Action 接受/拒绝时通过 Stream 推送中间状态，而非仅在 RPC 结束时返回单次响应。

### 4.7 故障排查

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 端口连接拒绝 | Bridge 未 `Start()` | 确认进程已启动并监听 |
| `grpcurl list` 失败 | 无 reflection | 提供 `.proto` 描述文件 |
| 指令无响应 | Handler 为 stub | 检查 `navigation_handler.cpp` 是否已接线 |
| 位姿偏移 | TF 未就绪 | 等待 `transform` 发布后再发 `NAV_CMD_START` |
| 消息过大 | 超过 100 MB | 减少 `PoseArray` 点数或拆分请求 |
| MQTT 无效果 | `use_mqtt=false` 且插件未实现 | 使用 gRPC 或等待 MQTT 插件 |

### 4.8 扩展自定义 Handler

1. 在 `external_command_service.proto` 中声明新 RPC
2. 使用 `DEFINE_HANDLER_SIGNATURE` + 继承 `RpcHandler`
3. 在 `GrpcBridgeServer` 构造函数中 `RegisterHandler<YourHandler>()`
4. 在 Handler 内通过 `GrpcBridgeContextInterface` 访问 Navigator / 地图等依赖

参考模板见 [06_grpc.md §6.5](06_grpc.md#65-自定义-handler)。
