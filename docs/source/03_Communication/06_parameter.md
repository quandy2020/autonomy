# 6. 参数 Parameter

运行时键值对，类似 ROS 2 Parameter Server。启动期大块算法 conf 仍走各域 `*.pb.txt`（见 [Framework · 配置](../05_Framework/04_configuration.md)）。 Parameter 适合运行时小改（开关 debug、微调系数）；各域大块算法配置仍以 `*.pb.txt` 为主（见 Framework 配置章）。若目标 Server 未暴露 Parameter API，改 pb 后仍需重启对应进程。

## 角色

| 角色 | 类 | 说明 |
|------|-----|------|
| Server | `ParameterServer` | 持有参数表，挂在 Node 上 |
| Client | `ParameterClient` | Get / Set / List |

服务名通常为 `global_parameter_service`（示例）。

## C++

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

## 配置分工

| 类型 | 落点 |
|------|------|
| 启动期算法 conf | 模块 `*.pb.txt` / Lua（Framework） |
| 运行时热改 | Parameter |
| QoS / 路径 | `AUTOLINK_*` 环境变量 |

示例：`examples/cpp/paramserver.cpp` · `examples/python/`。
