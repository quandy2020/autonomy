# 6. 参数 Parameter

跨节点键值配置。类似 ROS 2 Parameter Server。

## 6.1 角色

| 角色 | 类 | 说明 |
|------|-----|------|
| Server | `ParameterServer` | 持有参数表，挂在 Node 上 |
| Client | `ParameterClient` | Get / Set / List |

服务名通常为 `global_parameter_service`（示例）。

## 6.2 C++

```cpp
#include "autolink/parameter/parameter_server.hpp"
#include "autolink/parameter/parameter_client.hpp"

auto param_server = std::make_shared<ParameterServer>(node);
param_server->SetParameter(Parameter("max_speed", 1.5));
param_server->SetParameter(Parameter("frame_id", std::string("map")));

ParameterClient client(node, "global_parameter_service");
Parameter p;
client.GetParameter("max_speed", &p);
```

| 类型 | 构造 |
|------|------|
| double / int / bool / string | `Parameter(name, value)` |

## 6.3 配置分工

| 类型 | 落点 |
|------|------|
| 启动期算法 conf | 模块 `*.pb.txt` / Lua（Framework） |
| 运行时热改 | Parameter |
| QoS / 路径 | `AUTOLINK_*` 环境变量 |

示例：`examples/cpp/paramserver.cpp` · `examples/python/`。
