(bridge-guide)=
# 0. Bridge 指南

`autonomy/bridge` 在机载导航栈与场外客户端（移动端、云平台、调度）之间提供 gRPC 桥接，对标 `rosbridge_suite`、Nav2 gRPC 封装。

**定位**：外部接口层，翻译 Command、推送状态，**不参与**规划与控制 → [§2 架构](02_architecture.md) · [§3 RPC](rpcs/index.rst)

## 0.1 文档地图

| 角色 | 建议顺序 |
|------|----------|
| 新手 | 本页 [§0.2 快速开始](#02-快速开始) → [§2 架构](02_architecture.md) |
| **集成 / 调 API** | [rpcs/01 接入](rpcs/01_connection_guide.md) → [rpcs/13 测试用例](rpcs/13_integration_tests.md) |
| 机载部署 / 改配置 | [§1 参数配置](01_options.md)（[§0.3 配置入口](#03-配置入口)） |
| gRPC 开发 | [§2](02_architecture.md) → 源码旁 `autonomy/bridge/grpc/DESIGN.md` → [§4 gRPC](04_grpc.md) → [grpc/](grpc/index.rst) |
| 选型 | [§6 综述](06_survey.md) |

侧边栏 **§0–§6** 为模块主干；**§1** 为 `bridge.lua` 详表（不列入 toctree）；`rpcs/`、`grpc/` 为专题。MQTT 已移除，见 [§5](05_mqtt.md)。

---

## 0.2 快速开始

1. 编辑 `config/bridge/bridge.lua`（[§0.3](#03-配置入口) · [§1](01_options.md)）  
2. `CreateOptions` → `BridgeServer` → `Start()`  
3. 调 API — [rpcs/01 接入与验证](rpcs/01_connection_guide.md)

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
# 推荐：automsgs/tools/cli/rpc-cli.py；或 grpcurl 指定域 proto
grpcurl -plaintext -import-path $REPO -proto automsgs/proto/rpcs/system.proto \
  -d '{}' 127.0.0.1:5005 \
  automsgs.rpcs.system.SystemService/GetCapabilities
```

```{figure} images/01_running.png
:alt: grpcurl 探测 automsgs.rpcs
:align: center
:width: 85%
:name: fig-bridge-grpcurl
```

---

## 0.3 配置入口

`config/bridge/bridge.lua` / `conf/bridge.pb.txt` → `CreateOptions()` → `proto::BridgeOptions`。字段详表 → [§1 参数配置](01_options.md)。

| 资产 | 路径 |
|------|------|
| 运行时 Lua | `config/bridge/bridge_options.lua` |
| 默认 protobuf 文本 | `autonomy/bridge/conf/bridge.pb.txt` |
| 通道拓扑说明 | `autonomy/bridge/dag/bridge.dag` |
| 独立启动 | `autonomy/bridge/launch/bridge.launch` |

---

**导航**：[§2 架构 →](02_architecture.md) · [§3 RPC →](rpcs/index.rst)
