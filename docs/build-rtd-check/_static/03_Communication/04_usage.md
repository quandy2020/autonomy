# 4. 使用指南

本章给出 autolink 日常开发中的 API 用法、配置与排错要点。

## 4.1 初始化与关闭

```cpp
#include "autolink/common/init.hpp"

int main(int argc, char* argv[]) {
    // 必须在一切 API 之前调用
    if (!autolink::Init(argv[0])) {
        return 1;
    }

    // ... 业务逻辑 ...

    autolink::WaitForShutdown();  // 阻塞至 SIGINT
    return 0;
}
```

| API | 说明 |
|-----|------|
| `Init(binary_name, dag_info)` | 初始化框架；`dag_info` 可选，供日志标识 |
| `OK()` | 是否仍在运行（未收到 shutdown 信号） |
| `WaitForShutdown()` | 等待退出信号 |
| `Clear()` | 主动清理（通常析构时自动调用） |

## 4.2 创建 Node

```cpp
auto node = autolink::CreateNode("localization_server", "");
if (!node) {
    AERROR << "CreateNode failed: Init not called?";
    return;
}
```

| 参数 | 说明 |
|------|------|
| `node_name` | 全局唯一节点名 |
| `name_space` | 可选命名空间，完整名为 `/namespace/node_name` |

## 4.3 Writer（发布）

```cpp
// 简单方式
auto writer = node->CreateWriter<MsgT>("/channel/name");

// 带 RoleAttributes（QoS、类型名等）
autolink::proto::RoleAttributes attr;
attr.set_channel_name("/channel/name");
attr.mutable_qos_profile()->set_depth(10);
auto writer2 = node->CreateWriter<MsgT>(attr);

// 发布
writer->Write(msg);                    // 值或 shared_ptr
if (writer->HasReader()) { /* ... */ }
```

## 4.4 Reader（订阅）

```cpp
// 回调方式（推荐）
auto reader = node->CreateReader<MsgT>(
    "/channel/name",
    [](const std::shared_ptr<MsgT>& msg) { /* handle */ });

// ReaderConfig
autolink::ReaderConfig cfg;
cfg.channel_name = "/channel/name";
cfg.pending_queue_size = 10;
auto reader2 = node->CreateReader<MsgT>(cfg, callback);

// 无回调时可用 Observe 轮询（Component 内部机制）
```

| `pending_queue_size` | 行为 |
|----------------------|------|
| 1（默认） | 新消息覆盖未处理消息 |
| N > 1 | 缓存 N 条，满则丢弃最旧 |

## 4.5 Service / Client

**Service**

```cpp
auto service = node->CreateService<Request, Response>(
    "/calc",
    [](const std::shared_ptr<Request>& req,
       std::shared_ptr<Response>& resp) {
        resp = std::make_shared<Response>();
        resp->set_value(req->a() + req->b());
    });
```

**Client**

```cpp
auto client = node->CreateClient<Request, Response>("/calc");
client->WaitForService(std::chrono::seconds(5));

auto req = std::make_shared<Request>();
auto resp = client->SendRequest(req, std::chrono::seconds(3));

// 异步
auto fut = client->AsyncSendRequest(req);
auto resp2 = fut.get();
```

## 4.6 Action Server / Client

**Server**

```cpp
#include "autolink/action/action.hpp"

auto server = autolink::action::CreateServer<ActionT>(
    node, "/follow_path",
    // handle_goal
    [](const autolink::action::GoalUUID& id,
       std::shared_ptr<const typename ActionT::Goal> goal) {
        return autolink::action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    // handle_cancel
    [](std::shared_ptr<autolink::action::ServerGoalHandle<ActionT>> gh) {
        return autolink::action::CancelResponse::ACCEPT;
    },
    // handle_accepted — 应在新线程中 Execute
    [](std::shared_ptr<autolink::action::ServerGoalHandle<ActionT>> gh) {
        std::thread([gh]() {
            gh->Execute();
            // gh->PublishFeedback(...);
            // gh->Succeed(result);
        }).detach();
    });
```

**Client**

```cpp
auto client = autolink::action::CreateClient<ActionT>(node, "/follow_path");

autolink::action::Client<ActionT>::SendGoalOptions opts;
opts.feedback_callback = [](auto, auto fb) { /* ... */ };
opts.result_callback = [](const auto& wrapped) { /* ... */ };

auto gh = client->AsyncSendGoal(goal, opts).get();
auto result = client->AsyncGetResult(gh).get();
```

**ServerGoalHandle 终态方法**：`Succeed()` / `Canceled()` / `Abort()`

## 4.7 Parameter

```cpp
#include "autolink/parameter/parameter_client.hpp"

autolink::ParameterClient param_client(node, "global_param_server");

autolink::Parameter param("max_speed", 1.5);
param_client.SetParameter(param);

autolink::Parameter result;
param_client.GetParameter("max_speed", &result);
```

## 4.8 日志

```cpp
#include "autolink/common/log.hpp"

AINFO  << "info";
AWARN  << "warn";
AERROR << "error";
ADEBUG << "debug";  // 需开启 DEBUG 级别
```

日志异步写入，调度配置中可为 `async_log` 线程单独绑核。

## 4.9 Action 类型定义

Action proto 须满足：

1. `syntax = "proto3"`
2. 嵌套 `Goal`、`Feedback`、`Result`
3. 顶层字段 `goal = 1; feedback = 2; result = 3;`
4. 推荐独立 `XxxActionErrorCode` 枚举

详见 [§3.9](03_concepts.md#39-action-类型约束proto3) 与 Framework 文档中的完整 NavigateToPose 示例。

## 4.10 录制与回放

```bash
# 录制
autolink_recorder record -a -o /tmp/demo.record

# 查看信息
autolink_recorder info /tmp/demo.record

# 回放
autolink_recorder play -f /tmp/demo.record
```

C++ API：`RecordWriter` / `RecordReader` / `RecordViewer`。

## 4.11 与 commsgs 联用

```cpp
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/proto/sensor_msgs.pb.h"

// 算法内使用 struct
autonomy::commsgs::sensor_msgs::Imu imu;
imu.header.stamp = autolink::Time::Now();

// 若 Writer 模板为 proto 类型
autonomy::commsgs::proto::sensor_msgs::Imu proto;
autonomy::commsgs::sensor_msgs::ToProto(imu, &proto);
writer->Write(proto);

// 若 Writer 模板为 struct（项目内扩展）
writer->Write(imu);
```

> 具体模板参数以各模块 CMake 目标与 traits 注册为准；推荐边界显式 ToProto。

## 4.12 配置文件

| 文件 | 用途 |
|------|------|
| `autolink/conf/*.conf` | 调度策略（classic / choreography） |
| `*.dag` | 组件拓扑、channel 读写配置 |
| `*.launch` | 多 DAG 一键启动 |
| `transport_conf.pb.txt` | 传输后端参数 |

## 4.13 注意事项清单

1. **Init 顺序**：任何 `CreateNode` 之前必须 `Init`
2. **名称唯一**：node / channel 角色名在拓扑内不可冲突
3. **类型匹配**：Writer 与 Reader 的 `MessageT` 必须一致
4. **Action 非阻塞**：`handle_accepted` 中执行长任务务必开新线程
5. **线程安全**：API 多数线程安全，同一 Writer 并发 Write 需注意消息内容
6. **智能指针**：Node / Writer / Reader 均为 `shared_ptr`，无需手动释放

## 4.14 故障排查

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `CreateNode` 返回 null | 未 `Init` | 检查 `main` 入口 |
| Reader 无回调 | channel 名或类型不匹配 | `autolink_channel` 查看拓扑 |
| 跨进程不通 | SHM 权限 / 发现失败 | 检查 UDP 端口、防火墙 |
| Action 无反馈 | 未调用 `Execute()` | Server 端检查状态机 |
| 消息延迟大 | 调度 / 绑核不当 | 调整 [§8 调度](08_scheduler_transport.md) |
| 回放不一致 | 仿真时钟未同步 | 使用 `Clock` 模式回放 |

**调试工具**

| 工具 | 用途 |
|------|------|
| `autolink_monitor` | 实时查看 channel 消息 |
| `autolink_channel` | 列出 channel 与类型 |
| `autolink_recorder` | 录制复现 |

## 4.15 Python 快速用法

```python
import autolink
from autolink import Node

autolink.init("py_talker")
node = Node("py_talker")
writer = node.create_writer("/chatter", Chatter)

while autolink.ok():
    msg = Chatter()
    msg.content = "hello"
    writer.write(msg)
```

完整 API 见 `autolink/docs/source/autolink_python_api_cn.md`。
