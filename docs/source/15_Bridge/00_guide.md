(bridge-guide)=
# 0. Bridge 指南

`autonomy/bridge`：机载栈与场外客户端之间的 gRPC 桥。翻译 Command、推送状态，**不参与**规划与控制。

### 0.2 快速开始

1. 编辑 `config/bridge/bridge.lua`（[§1](01_options.md)）
2. `CreateOptions` → `BridgeServer` → `Start()`
3. 调 API — [rpcs/01](rpcs/01_connection_guide.md)

```cpp
auto options = autonomy::bridge::CreateOptions("bridge.pb.txt");
auto server = std::make_shared<autonomy::bridge::BridgeServer>(options);
server->Start();
server->WaitForShutdown();
```

```lua
AUTONOMY_BRIDGE = {
    grpc = { host = "127.0.0.1", port = 5005, num_grpc_threads = 5 },
}
```

```bash
grpcurl -plaintext -import-path $REPO -proto automsgs/proto/rpcs/system.proto \
  -d '{}' 127.0.0.1:5005 \
  automsgs.rpcs.system.SystemService/GetCapabilities
```

```{figure} images/01_running.png
:alt: grpcurl 探测
:align: center
:width: 85%
:name: fig-bridge-grpcurl
```

### 0.3 配置入口

| 资产 | 路径 |
|------|------|
| Lua | `config/bridge/bridge_options.lua` |
| 默认 pb | `autonomy/bridge/conf/bridge.pb.txt` |
| DAG | `autonomy/bridge/dag/bridge.dag` |
| Launch | `autonomy/bridge/launch/bridge.launch` |

字段详表 → [§1](01_options.md)。
