# 4. 服务 Service

Service 提供 **同步请求–响应 RPC**。对标 ROS 2 Service。

| 本文 §4 | 相关文档 |
|---------|----------|
| RPC | [§0 指南](00_guide.md) · [§2 节点 Node](02_node.md) · [§6 参数 Parameter](06_parameter.md) |

---

## 4.1 调用流程

Client 阻塞至 Server 回调填好 `resp` 或超时。与 Channel 区别：Channel 为持续流、无应答；Service 适合地图查询、一次性配置读取等短事务。长时、可取消任务用 [Action](05_action.md)。

<div class="comm-flow-diagram">
<div class="comm-flow-header">
  <span class="comm-flow-badge">RPC</span>
  <span class="comm-flow-title">Client::SendRequest ↔ Service 回调</span>
  <span class="comm-flow-sub">Discovery 匹配 <strong>service 名 + Req/Resp 类型</strong> 后，<code>SendRequest</code> 同步阻塞至 <code>resp</code> 或超时</span>
</div>

<div class="comm-flow-pipeline">
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-label">Client</span>
    <span class="comm-flow-step-title">SendRequest</span>
    <span class="comm-flow-step-sub"><code>SendRequest(req)</code> · 超时变体</span>
  </div>
  <div class="comm-flow-link">
    <span class="comm-flow-link-label">Service RPC</span>
    <span class="comm-flow-link-arrow" aria-hidden="true">↔</span>
  </div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-label">Service</span>
    <span class="comm-flow-step-title">回调(req, resp)</span>
    <span class="comm-flow-step-sub">handler 同步执行 · 填充 resp</span>
  </div>
</div>

<table class="comm-flow-legend">
  <tr><th>要点</th><th>说明</th></tr>
  <tr><td>注册</td><td><code>CreateClient</code> / <code>CreateService</code> 并行注册，Discovery 匹配后调用</td></tr>
  <tr><td>命名</td><td><code>service_name</code> 拓扑内唯一；Req/Resp 类型须一致</td></tr>
  <tr><td>失败</td><td><code>SendRequest</code> 返回 <code>nullptr</code>：Server 未就绪或名/类型不匹配</td></tr>
</table>
</div>

---

## 4.2 C++ 示例

`autolink/examples/cpp/service.cpp` 在同进程创建双 Node，循环自测：

```cpp
using autolink::examples::Driver;

const std::string service_name = "test_server_" + suffix;
auto server_node = autolink::CreateNode("service_server_" + suffix, "/examples");
auto client_node = autolink::CreateNode("service_client_" + suffix, "/examples");

// Server：回调签名 (const shared_ptr<Req>&, shared_ptr<Resp>&)
auto server = server_node->CreateService<Driver, Driver>(
    service_name,
    [](const std::shared_ptr<Driver>& request,
       std::shared_ptr<Driver>& response) {
        AINFO << "server: request id: " << request->msg_id();
        response->set_msg_id(request->msg_id());
        response->set_timestamp(autolink::Time::Now().ToNanosecond());
    });

auto client = client_node->CreateClient<Driver, Driver>(service_name);
if (server == nullptr || client == nullptr) {
    AERROR << "failed to create service/client.";
    return 1;
}

while (autolink::OK()) {
    auto driver_msg = std::make_shared<Driver>();
    driver_msg->set_msg_id(req_id++);
    auto res = client->SendRequest(driver_msg);   // 阻塞至响应或超时
    if (res != nullptr) {
        AINFO << "client: response msg_id=" << res->msg_id();
    } else {
        AINFO << "client: service may not ready.";
    }
    sleep(1);
}
```

**消息类型**（`examples.proto`）：

```protobuf
message Driver {
  string content = 1;
  uint64 msg_id = 2;
  uint64 timestamp = 3;
}
```

运行：`./autolink_example_service`（单进程自包含）。

---

## 4.3 CreateService / SendRequest

`Node::CreateService` / `CreateClient` 注册 RPC 对；Client 侧 `SendRequest` 同步阻塞至响应或超时：

| API | 说明 |
|-----|------|
| `CreateService<Req,Resp>(name, handler)` | 注册服务；handler 同步执行 |
| `CreateClient<Req,Resp>(name)` | 创建客户端 |
| `SendRequest(req)` | 同步等待响应；失败返回 `nullptr` |
| `SendRequest(req, timeout)` | 带超时的同步调用 |

**命名**：`service_name` 在拓扑内唯一；多实例测试时常加 PID 后缀（见上例）。`namespace` 通过 Node 第二参数传入（`"/examples"`）。

---

## 4.4 Python API

**Server**（`py_service.py`）— 回调直接返回响应对象：

```python
def callback(data):
    print("get Request [", data, "]")
    return ChatterBenchmark(content="svr: Hello client!", seq=data.seq + 2)

node = autolink.Node("service_node")
node.create_service("server_01", ChatterBenchmark, ChatterBenchmark, callback)
node.spin()
```

**Client**（`py_client.py`）：

```python
node = autolink.Node("client_node")
client = node.create_client("server_01", ChatterBenchmark, ChatterBenchmark)
while not autolink.is_shutdown():
    req = ChatterBenchmark()
    req.content = "clt:Hello service!"
    req.seq = count
    response = client.send_request(req)
    print("get Response [", response, "]")
    time.sleep(1)
```

```bash
# terminal 1
python3 py_service.py

# terminal 2
python3 py_client.py
```

---

## 4.5 Parameter

ParameterServer 内部用多组 Service 实现 `SetParameter` / `GetParameter` / `ListParameters`（见 [§6](06_parameter.md)）。业务 Service 与之独立，勿占用 Parameter 保留的 node 名。

---

**导航**：[← §3 通道 Channel](03_channel.md) · [§0 指南](00_guide.md) · [§5 动作 Action →](05_action.md)
