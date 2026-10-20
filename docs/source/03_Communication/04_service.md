# 4. 服务 Service

同步请求–响应 RPC。类似 ROS 2 Service。

## 4.1 流程

```text
Client.SendRequest(req) → 传输 → Server handler → 返回 Response
```

| API | 说明 |
|-----|------|
| `CreateService<Req,Resp>(name, handler)` | 服务端 |
| `CreateClient<Req,Resp>(name)` | 客户端 |
| `client->SendRequest(req[, timeout])` | 同步调用 |

同进程演示用不同 Node 名（PID 后缀），见 `examples/cpp/service.cpp`。

## 4.2 C++ 示例

```cpp
auto server = server_node->CreateService<Driver, Driver>(
    service_name,
    [](const std::shared_ptr<Driver>& req,
        std::shared_ptr<Driver>& resp) {
        resp->set_content("svr: " + req->content());
        resp->set_msg_id(req->msg_id());
});

auto client = client_node->CreateClient<Driver, Driver>(service_name);
auto resp = client->SendRequest(req);
```

## 4.3 Python

```python
def callback(data):
return ChatterBenchmark(content="svr: Hello client!", ...)
node.create_service(service_name, ChatterBenchmark, ChatterBenchmark, callback)
# client: node.create_client(...); client.send_request(req)
```

示例：`examples/python/py_service.py` · `py_client.py`。
