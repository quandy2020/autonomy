# 6. Parameter

Parameter 提供**跨节点键值配置**。对标 ROS 2 Parameter Server。

| 本文 §6 | 相关文档 |
|---------|----------|
| 全局参数 | [§0 指南](00_guide.md) · [§4 Service](04_service.md) · [Framework §7](../05_Framework/07_commsgs_integration.md) |

---

## 6.1 设计

Parameter 通过 **Service RPC** 在 Client 与 Server 之间读写键值；每个 Server 绑定一个 Node 名。

<div class="comm-flow-diagram">
<div class="comm-flow-header">
  <span class="comm-flow-badge">配置</span>
  <span class="comm-flow-title">ParameterClient → ParameterServer</span>
  <span class="comm-flow-sub">Client 构造时传入 <strong>目标 Server 的 node 名</strong>，非自己的 node 名</span>
</div>

<div class="comm-flow-pipeline">
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-label">Client</span>
    <span class="comm-flow-step-title">ParameterClient</span>
    <span class="comm-flow-step-sub">SetParameter · GetParameter · ListParameters</span>
  </div>
  <div class="comm-flow-link">
    <span class="comm-flow-link-label">Service RPC</span>
    <span class="comm-flow-link-arrow" aria-hidden="true">→</span>
  </div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-label">Server</span>
    <span class="comm-flow-step-title">ParameterServer</span>
    <span class="comm-flow-step-sub">键值存储，绑定 Node 名</span>
  </div>
</div>

<table class="comm-flow-legend">
  <tr><th>要点</th><th>说明</th></tr>
  <tr><td>Server 绑定</td><td>每个 <code>ParameterServer</code> 对应一个 Node 名（如 <code>"parameter"</code>）</td></tr>
  <tr><td>典型部署</td><td>进程内通常一个全局 Server；多 Server 须不同 node 名</td></tr>
</table>
</div>

---

## 6.2 C++ 示例

`autolink/examples/cpp/paramserver.cpp`：

```cpp
#include "autolink/parameter/parameter_client.hpp"
#include "autolink/parameter/parameter_server.hpp"

using autolink::Parameter;
using autolink::ParameterClient;
using autolink::ParameterServer;

int main(int argc, char** argv) {
    autolink::Init(argv[0]);
    auto node = autolink::CreateNode("parameter", "/examples");

    auto param_server = std::make_shared<ParameterServer>(node);
    auto param_client = std::make_shared<ParameterClient>(node, "parameter");

    // Server 侧写入
    param_server->SetParameter(Parameter("int", 1));
    Parameter parameter;
    param_server->GetParameter("int", &parameter);
    AINFO << "int: " << parameter.AsInt64();

    // Client 侧读写（跨节点时 client 用另一 Node）
    param_client->SetParameter(Parameter("string", "test"));
    param_client->GetParameter("string", &parameter);
    AINFO << "string: " << parameter.AsString();
    param_client->GetParameter("int", &parameter);
    AINFO << "int: " << parameter.AsInt64();
    return 0;
}
```

运行：`./autolink_example_paramserver`。

---

## 6.3 API

**Parameter 包装值**：

```cpp
Parameter("max_speed", 1.5);           // double
Parameter("frame_id", std::string("map"));
Parameter("enabled", true);
Parameter("int", 42);
```

**ParameterServer**（`parameter_server.hpp`）：

| 方法 | 说明 |
|------|------|
| `SetParameter(p)` | 设置或覆盖 |
| `GetParameter(name, &out)` | 按名读取；不存在返回 false |
| `ListParameters(&vec)` | 枚举全部 |

**ParameterClient**：

| 方法 | 说明 |
|------|------|
| `SetParameter(p)` | 向远端 Server 写入 |
| `GetParameter(name, &out)` | 从远端读取 |
| `ListParameters(&vec)` | 列举远端参数 |

**读取辅助**：`AsInt64()`、`AsDouble()`、`AsString()`、`AsBool()`。

---

## 6.4 Python

`py_parameter.py`：

```python
from autolink_py3 import parameter

PARAM_SERVICE_NAME = "global_parameter_service"

test_node = autolink.Node(PARAM_SERVICE_NAME)
srv = parameter.ParameterServer(test_node)

clt = parameter.ParameterClient(test_node, PARAM_SERVICE_NAME)
clt.set_parameter(parameter.Parameter("author_name", "WanderingEarth"))
clt.set_parameter(parameter.Parameter("author_age", 5000))
clt.set_parameter(parameter.Parameter("author_score", 888.88))

param_list = clt.get_paramslist()
for p in param_list:
    print(p.debug_string())
```

```bash
python3 py_parameter.py
```

---

## 6.5 与配置文件分工

| 机制 | 适用 |
|------|------|
| Lua / YAML 配置 | 启动时固定参数、模块装配 |
| Parameter | 运行时在线调参、多节点共享 |
| Component DAG `config` | 模块级 channel 名、间隔等 |

Autonomy 模块参数管线见 [Framework §7](../05_Framework/07_commsgs_integration.md)。

---

## 6.6 排错

| 现象 | 处理 |
|------|------|
| Get 失败 | Server 未启动；Client 目标 node 名与 Server Node 名不一致 |
| 值未同步 | 确认 Set 在 Server 侧还是 Client 侧；List 核对键名 |
| 多进程 | Server 进程须先 `Init` 并创建 ParameterServer Node |

---

**导航**：[← §5 Action](05_action.md) · [§0 指南](00_guide.md) · [§7 Plugin →](07_plugin.md)
