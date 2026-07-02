# 2. 快速开始

### 2.1 三步启用

1. 编辑 `config/bridge/bridge.lua`，设置 `use_grpc` / `use_mqtt` 及监听地址
2. 在应用代码中加载配置并构造 `BridgeServer`（`config/autonomy.lua` 根表集成待完成）
3. 调用 `Start()` 启动 gRPC 服务，外部客户端连接 `host:port`

### 2.2 最小 C++ 示例

```cpp
#include "autonomy/bridge/bridge_server.hpp"
#include "autonomy/bridge/common/bridge_interface.hpp"
#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"

// 1. 加载 Lua 配置
autonomy::common::ConfigurationFileResolver resolver({"config"});
auto lua_code = resolver.GetFileContentOrDie("bridge/bridge.lua");
auto dict = autonomy::common::LuaParameterDictionary::NonReferenceCounted(
    lua_code, &resolver);
auto bridge_dict = dict->GetDictionary("AUTONOMY_BRIDGE").get();
auto options = autonomy::bridge::common::LoadOptions(bridge_dict);

// 2. 启动 Bridge
auto server = std::make_shared<autonomy::bridge::BridgeServer>(options);
server->Start();
server->WaitForShutdown();  // 阻塞直至 Shutdown
```

### 2.3 默认配置

```lua
-- config/bridge/bridge.lua
AUTONOMY_BRIDGE = {
    use_grpc = true,
    use_mqtt = false,
    grpc = {
        host = "127.0.0.1",
        port = 5005,
        num_grpc_threads = 5,
        num_event_threads = 5,
    },
    mqtt = {
        host = "127.0.0.1",
        port = 12345,
    },
}
```

> **注意**：当前 `GrpcBridgeServer` 构造函数中监听地址硬编码为 `127.0.0.1`，线程数为 4；`bridge.lua` 中的 `host`/`port` 尚未完全接线，见 [§4.1](04_usage.md#41-配置)。

### 2.4 客户端连通性测试

使用 `grpcurl` 验证服务是否可达（需先生成并安装 `external_command_service.proto` 描述）：

```bash
# 列出服务（需 reflection 或 proto 描述文件）
grpcurl -plaintext 127.0.0.1:5005 list

# 发送导航停止指令（示例）
grpcurl -plaintext -d '{"command": 2}' \
  127.0.0.1:5005 \
  autonomy.bridge.proto.AutonomyService/SendNavigationCommand
```

### 2.5 与 Autonomy 主进程联调

完整联调需同时启动导航栈与 Bridge：

```
终端 1：启动 Autonomy 核心（map / localization / planning / control / navigator）
终端 2：启动 BridgeServer，外部客户端通过 gRPC 下发导航目标
```

当前 `bridge_main.cpp` 仅为版本打印 stub，生产环境建议在 `system::Autonomy` 中嵌入 `BridgeServer`，或完善独立 `bridge_server` 二进制。
