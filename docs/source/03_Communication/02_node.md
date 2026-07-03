# 2. Node

Node 是 autolink 的**通信句柄**：进程内创建 Writer/Reader、Service/Client、Action、Parameter 等端点的统一入口。对标 ROS 2 `rclcpp::Node`、Cyber RT `cyber::Node`。

| 本文 §2 | 相关文档 |
|---------|----------|
| Node 通信句柄 | [§0 指南](00_guide.md) · [§3 Channel](03_channel.md) · [§1 架构](01_architecture.md) |

---

## 2.1 设计

Node 将 Channel 与 Service 能力聚合在 `NodeChannelImpl` 与 `NodeServiceImpl` 中；Action / Parameter 经独立 API 挂载到同一 Node。

<div class="comm-flow-diagram">
<div class="comm-flow-header">
  <span class="comm-flow-badge">Node</span>
  <span class="comm-flow-title">CreateNode → 端点工厂</span>
  <span class="comm-flow-sub">同一拓扑内 <strong>node 名、reader/writer、service/client 名均不可重复</strong>（<code>node.hpp</code>）</span>
</div>

<div class="comm-flow-pipeline comm-flow-pipeline--hub">
  <div class="comm-flow-step comm-flow-step-mid comm-flow-step-hub">
    <span class="comm-flow-step-label">入口</span>
    <span class="comm-flow-step-title">CreateNode(name, ns)</span>
  </div>
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-label">Channel</span>
    <span class="comm-flow-step-title">Writer / Reader</span>
    <span class="comm-flow-step-sub">持续数据流</span>
  </div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-label">RPC</span>
    <span class="comm-flow-step-title">Service / Client</span>
  </div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-label">扩展</span>
    <span class="comm-flow-step-title">Action / Parameter</span>
    <span class="comm-flow-step-sub">独立 API</span>
  </div>
</div>

<div class="comm-flow-pipeline comm-flow-pipeline--chain">
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">Init</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">CreateNode</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-title">创建端点</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">OK / WaitForShutdown</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-title">析构 / Clear</span>
  </div>
</div>

<div class="comm-flow-foot"><code>Init</code> 加载 <code>AUTOLINK_PATH/conf/autolink.pb.conf</code>；未 <code>Init</code> 时 <code>CreateNode</code> 返回 <code>nullptr</code>。</div>
</div>

---

## 2.2 创建 Node

```cpp
#include "autolink/autolink.hpp"

int main(int argc, char* argv[]) {
    if (!autolink::Init(argv[0]))   // 失败时返回 false
        return 1;

    // 无 namespace
    auto talker = autolink::CreateNode("talker");

    // 带 namespace（service 等同理可继承）
    auto srv = autolink::CreateNode("service_server", "/examples");

    autolink::WaitForShutdown();    // SIGINT 前阻塞
    return 0;
}
```

| API | 说明 |
|-----|------|
| `Init(argv[0])` | 进程级初始化，仅需一次 |
| `CreateNode(name)` | 创建 Node，`name` 全局唯一 |
| `CreateNode(name, namespace)` | 可选命名空间前缀 |
| `OK()` | 运行时状态，主循环中检查 |
| `WaitForShutdown()` | 等待退出信号 |
| `node->Name()` | 获取节点名 |

**Talker 完整示例**（`autolink/examples/cpp/talker.cpp`）：

```cpp
int main(int argc, char* argv[]) {
    if (!autolink::Init(argv[0]))
        return 1;
    auto talker_node = autolink::CreateNode("talker");
    auto talker = talker_node->CreateWriter<Chatter>("channel/chatter");
    Rate rate(1.0);
    uint64_t seq = 0;
    while (autolink::OK()) {
        auto msg = std::make_shared<Chatter>();
        msg->set_timestamp(Time::Now().ToNanosecond());
        msg->set_seq(seq);
        msg->set_content("Hello, autolink!");
        talker->Write(msg);
        AINFO << "talker sent a message! No. " << seq;
        ++seq;
        rate.Sleep();
    }
    return 0;
}
```

**Listener**（`listener.cpp`）仅创建 Reader 后 `WaitForShutdown`，回调由 Scheduler 触发：

```cpp
auto listener_node = autolink::CreateNode("listener");
listener_node->CreateReader<Chatter>("channel/chatter", MessageCallback);
autolink::WaitForShutdown();
```

---

## 2.3 Node 提供的端点 API

| 方法 | 返回类型 | 专题 |
|------|----------|------|
| `CreateWriter<MessageT>(channel)` | `Writer<MessageT>` | [§3](03_channel.md) |
| `CreateReader<MessageT>(channel, cb)` | `Reader<MessageT>` | [§3](03_channel.md) |
| `CreateReader<MessageT>(ReaderConfig, cb)` | `Reader<MessageT>` | [§3](03_channel.md) |
| `CreateService<Req,Resp>(name, handler)` | `Service` | [§4](04_service.md) |
| `CreateClient<Req,Resp>(name)` | `Client` | [§4](04_service.md) |
| `DeleteReader` / `DeleteWriter` | `bool` | 动态拆除端点 |

Action 与 Parameter 不挂在 `Node` 成员函数上，但构造时传入 `std::shared_ptr<Node>`：

```cpp
auto server = autolink::action::CreateServer<Traits>(node, action_name, ...);
auto param_server = std::make_shared<ParameterServer>(node);
```

---

## 2.4 多 Node 同进程

`service.cpp` 在同一进程创建 **server Node** 与 **client Node**，避免读写端点名冲突，并演示 RPC 自测：

```cpp
const std::string suffix = std::to_string(getpid()) + "_"
    + std::to_string(autolink::Time::Now().ToNanosecond());
auto server_node = autolink::CreateNode("service_server_" + suffix, "/examples");
auto client_node = autolink::CreateNode("service_client_" + suffix, "/examples");
```

Action 示例同样用 PID+时间戳生成唯一 node 名（`action_listener.cpp` / `action_talker.cpp`）。

---

## 2.5 Component 模式下的 Node

继承 `Component` 时，框架在 `Initialize()` 前注入 `node_`，**不要**在 `Init()` 里 `CreateNode`：

```cpp
class TimerComponentSample : public TimerComponent {
    bool Init() override {
        driver_writer_ = node_->CreateWriter<Driver>("/carstatus/channel");
        return true;
    }
};
```

`node_` 由 `mainboard` 根据 DAG 中 `config.name` 创建。见 [§10 Component](10_component.md)。

---

## 2.6 Python

```python
from autolink_py3 import autolink

autolink.init("talker_sample")          # 可传进程名
node = autolink.Node("node_name1")
writer = node.create_writer("channel/chatter", Chatter, 6)  # 第 3 参为 QoS depth
reader = node.create_reader("channel/chatter", Chatter, callback)
node.spin()                             # 阻塞处理回调
autolink.shutdown()
```

| C++ | Python |
|-----|--------|
| `Init` / `WaitForShutdown` | `init` / `shutdown` / `spin` |
| `OK()` | `is_shutdown()` 取反 |
| `CreateWriter<T>(ch)` | `create_writer(ch, MsgType, depth)` |

示例：`autolink/examples/python/py_talker.py`、`py_listener.py`。

**双进程 Talker/Listener**：

```bash
export AUTOLINK_PATH=<含 conf/autolink.pb.conf 的路径>
cd build/autolink/bin/examples
./autolink_example_listener    # terminal 1
./autolink_example_talker      # terminal 2
```

---

## 2.7 排错

| 现象 | 原因与处理 |
|------|------------|
| `CreateNode` 为 null | 未调用 `Init` 或 `Init` 失败 |
| 拓扑冲突 | 重复 `node_name`；换名或杀残留进程 |
| Python 找不到模块 | 设置 `PYTHONPATH`（见 `examples/python/README.md`） |

---

**导航**：[← §1 架构](01_architecture.md) · [§0 指南](00_guide.md) · [§3 Channel →](03_channel.md)
