(bridge-guide)=
# 0. Bridge 指南

## 背景

场外客户端要发 Navigate、手机要看状态——需要 **gRPC 桥** 把 Command 转进机载栈。`autonomy/bridge` **不参与**规划和控制算法；MQTT 已移除。若 CMake 未开 **`BUILD_GRPC=ON`**，整个 bridge 域可能被跳过，grpcurl 连不上是预期。

## 思路

- 契约以 **automsgs.rpcs** 为准；文档可用 `python3 docs/scripts/check_bridge_rpc_docs.py` 校验。  
- **先 GetCapabilities** — 确认进程与 proto 路径，再打 Navigate 等重命令。  
- **Command 是编排入口** — 真正跑 BT / planning 的是 task 侧；Bridge 只转发。  
- **Handler 接线进度不一** — 以 `grpc/07_handlers` 与实测为准。  
- 可视化不走 Bridge → [13 Visualization](../13_Visualization/00_guide.md)。

配置落点：

| 资产 | 路径 |
|------|------|
| Lua | `config/bridge/bridge_options.lua` |
| 默认 pb | `autonomy/bridge/conf/bridge.pb.txt` |
| DAG | `autonomy/bridge/dag/bridge.dag` |
| Launch | `autonomy/bridge/launch/bridge.launch` |


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 参数](01_options.md) | 端口、线程、字段表 |
| [02 架构](02_architecture.md) | 与 task / 各域关系 |
| [03 RPC 协议](03_rpc_protocol.md) | 协议总览 |
| [04 gRPC](04_grpc.md) | 实现与扩展 Handler |
| [05 综述](05_survey.md) | 选型边界 |
| [rpcs/](rpcs/01_connection_guide.md) | 联调手册 |

**联调 API** → [rpcs/01](rpcs/01_connection_guide.md) → 各 Command 页 → [集成测试](rpcs/12_integration_tests.md)。  
**改端口** → [01](01_options.md)。

### 2. 最短跑通

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

### 3. 常见坑

| 现象 | 先查 |
|------|------|
| grpcurl 连不上 | 端口；进程是否起；`BUILD_GRPC` |
| proto 找不到 | `-import-path` / `-proto` 是否指向仓库 |
| 命令成功但车不动 | task / planning / control 是否在跑 |
| 找 MQTT 文档 | 已删除 |
