# 2. 节点 Node

通信句柄：创建 Writer/Reader、Service/Client、Action、Parameter 的统一入口。类似 `rclcpp::Node` / `cyber::Node`。

## 2.1 定义与约束

| 约束 | 说明 |
|------|------|
| 先 `Init()` | 未初始化时 `CreateNode` 返回 `nullptr` |
| `node_name` 唯一 | 拓扑内不可重复；多实例加 PID 后缀 |
| 配置 | `Init` 加载 `AUTOLINK_PATH/conf/autolink.pb.conf` |

内部：`NodeChannelImpl` + `NodeServiceImpl`；Action / Parameter 经独立 API 挂到同一 Node。

## 2.2 创建与生命周期

```cpp
#include "autolink/autolink.hpp"

int main(int argc, char* argv[]) {
    if (!autolink::Init(argv[0]))
    return 1;
    auto talker = autolink::CreateNode("talker");
    auto srv = autolink::CreateNode("service_server", "/examples");
    autolink::WaitForShutdown();
    return 0;
}
```

| API | 说明 |
|-----|------|
| `Init(argv[0])` | 进程级，仅一次 |
| `CreateNode(name[, ns])` | `name` 全局唯一 |
| `OK()` / `WaitForShutdown()` | 主循环 / 阻塞退出 |

Talker（`examples/cpp/talker.cpp`）：

```cpp
auto node = autolink::CreateNode("talker");
auto writer = node->CreateWriter<Chatter>("channel/chatter");
Rate rate(1.0);
while (autolink::OK()) {
    auto msg = std::make_shared<Chatter>();
    msg->set_seq(seq++);
    writer->Write(msg);
    rate.Sleep();
}
```

Listener：`CreateReader` 后 `WaitForShutdown()`，回调由 Scheduler 触发。

## 2.3 Create* API

| 方法 | 专题 |
|------|------|
| `CreateWriter` / `CreateReader` | [§3](03_channel.md) |
| `CreateService` / `CreateClient` | [§4](04_service.md) |
| `action::CreateServer` / Parameter | [§5](05_action.md) · [§6](06_parameter.md) |

同进程多 Node：用 PID+时间戳后缀避免名冲突（见 `service.cpp`、`action_*.cpp`）。

## 2.4 Component

`Init()` 内用已注入的 `node_`，**不要**再 `CreateNode`：

```cpp
bool Init() override {
    writer_ = node_->CreateWriter<Driver>("/carstatus/channel");
    return true;
}
```

## 2.5 Python

```python
from autolink_py3 import autolink
autolink.init("talker_sample")
node = autolink.Node("node_name1")
writer = node.create_writer("channel/chatter", Chatter, 6)
node.spin()
autolink.shutdown()
```

```bash
export AUTOLINK_PATH=<含 conf 的路径>
./autolink_example_listener # terminal 1
./autolink_example_talker # terminal 2
```
