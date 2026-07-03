# 5. Action

Action 用于**长时任务**：Goal、Feedback、Result，支持取消与抢占。对标 ROS 2 Action。

| 本文 §5 | 相关文档 |
|---------|----------|
| 长任务 | [§0 指南](00_guide.md) · [§2 Node](02_node.md) · [commsgs Action](../14_Commsgs/08_nav_planning_msgs.md) |

Python Action 示例尚未实现（`py_action_server.py` 为占位）；以下以 C++ 与 `examples.proto` 为准。

---

## 5.1 设计

Action 在 Channel 之上封装 **Goal / Feedback / Result** 多路语义（与 ROS 2 类似），支持取消与抢占。

<div class="comm-flow-diagram">
<div class="comm-flow-header">
  <span class="comm-flow-badge">长任务</span>
  <span class="comm-flow-title">Client ↔ Server · Goal / Feedback / Result</span>
  <span class="comm-flow-sub">Channel 上封装三种消息（<strong>Traits</strong> 聚合）；异步 Goal、流式 Feedback，可取消与抢占</span>
</div>

<div class="comm-flow-pipeline">
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-label">Client</span>
    <span class="comm-flow-step-title">AsyncSendGoal</span>
    <span class="comm-flow-step-sub">AsyncGetResult · AsyncCancelGoal</span>
  </div>
  <div class="comm-flow-link">
    <span class="comm-flow-link-label">Goal · Feedback · Result</span>
    <span class="comm-flow-link-arrow" aria-hidden="true">↔</span>
  </div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-label">Server</span>
    <span class="comm-flow-step-title">工作线程执行</span>
    <span class="comm-flow-step-sub">PublishFeedback · SucceededCurrent</span>
  </div>
</div>

<table class="comm-flow-legend">
  <tr><th>要点</th><th>说明</th></tr>
  <tr><td>Traits</td><td>聚合 Goal / Feedback / Result 三种 protobuf（见下方示例）</td></tr>
  <tr><td>与 Service</td><td>Service 同步阻塞一轮；Action 异步 + 流式 Feedback</td></tr>
  <tr><td>取消</td><td><code>AsyncCancelGoal</code> → <code>IsCancelRequested</code> → <code>TerminateCurrent</code></td></tr>
  <tr><td>抢占</td><td>新 Goal → <code>IsPreemptRequested</code> → <code>AcceptPendingGoal</code></td></tr>
</table>
</div>

```protobuf
message SimpleMessageAction {
    message Goal   { string text = 1; }
    message Feedback { int32 index = 1; }
    message Result { bool success = 1; }
}
```

```cpp
struct SimpleMessageActionTraits {
    using Goal = ae::SimpleMessageAction_Goal;
    using Feedback = ae::SimpleMessageAction_Feedback;
    using Result = ae::SimpleMessageAction_Result;
};
```

Autonomy 导航 Action 定义见 [commsgs Action](../14_Commsgs/08_nav_planning_msgs.md)。

---

## 5.2 Server：`SimpleActionServer`

`action_listener.cpp` 使用 `SimpleActionServer`，在 goal 被接受后于**工作线程**执行：

```cpp
constexpr char kActionName[] = "examples/simple_message_action";

void RunAcceptedGoal(const std::shared_ptr<ActionServer>& server) {
    GoalPtr goal = server->GetCurrentGoal();
    AINFO << "Executing goal, text=\"" << goal->text() << "\"";

    for (int i = 0; i < kFeedbackSteps; ++i) {
        if (server->IsCancelRequested()) {
            ResultPtr aborted = std::make_shared<Result>();
            aborted->set_success(false);
            server->TerminateCurrent(aborted);
            return;
        }
        if (server->IsPreemptRequested()) {
            server->AcceptPendingGoal();   // 切换至 pending goal
            break;
        }
        FeedbackPtr fb = std::make_shared<Feedback>();
        fb->set_index(i);
        server->PublishFeedback(fb);
        std::this_thread::sleep_for(kFeedbackPeriod);
    }

    ResultPtr ok = std::make_shared<Result>();
    ok->set_success(true);
    server->SucceededCurrent(ok);
}

int main(int argc, char* argv[]) {
    autolink::Init(argv[0]);
    auto node = autolink::CreateNode(node_name);
    std::shared_ptr<ActionServer> server;
    server = std::make_shared<ActionServer>(
        node, kActionName,
        [&server]() { RunAcceptedGoal(server); });  // execute 回调
    autolink::WaitForShutdown();
}
```

**关键 API**：

| 方法 | 用途 |
|------|------|
| `GetCurrentGoal()` | 当前执行中的 goal |
| `PublishFeedback(fb)` | 推送进度 |
| `IsCancelRequested()` | 客户端请求取消 |
| `IsPreemptRequested()` | 新 goal 抢占 |
| `AcceptPendingGoal()` | 切换到 pending goal |
| `SucceededCurrent(result)` | 成功结束 |
| `TerminateCurrent(result)` | 取消 / 异常结束 |

---

## 5.3 Client：异步 Goal + Future

`action_talker.cpp` 演示完整客户端流程：

```cpp
auto client = autolink::action::CreateClient<SimpleMessageActionTraits>(
    node, kActionName);

// 1. 等待 Server 就绪
while (autolink::OK() && !client->ActionServerIsReady()) {
    std::this_thread::sleep_for(kWaitServerReady);
}

// 2. 构造 Goal 与回调选项
SimpleMessageActionTraits::Goal goal;
goal.set_text("hello from action_talker");

ActionClient::SendGoalOptions opts;
opts.goal_response_callback = [](std::shared_ptr<GoalHandle> gh) {
    if (!gh) AWARN << "Goal rejected by server";
};
opts.feedback_callback = [](auto gh, auto fb) {
    if (fb) AINFO << "Feedback index=" << fb->index();
};

// 3. 发送并等待接受
const auto accepted_future = client->AsyncSendGoal(goal, opts);
accepted_future.wait_for(kAcceptTimeout);
std::shared_ptr<GoalHandle> handle = accepted_future.get();

// 4. 等待最终结果
auto result_future = handle->AsyncGetResult();
result_future.wait_for(kResultTimeout);
LogActionOutcome(handle, result_future.get());
```

`GoalHandle::WrappedResult` 含终端状态：`SUCCEEDED`、`CANCELED`、`ABORTED` 等，可用 `autolink::action::ToString(wr.code)` 打印。

---

## 5.4 运行

```bash
export AUTOLINK_PATH=...
cd build/autolink/bin/examples

# terminal 1 — 必须先启 Server
./action_listener

# terminal 2
./action_talker
```

预期日志：Server 周期 `feedback index=0..N`，Client 打印 `Goal accepted` 与 `Action terminal: SUCCEEDED`。

---

## 5.5 实现注意

- **Execute 在工作线程**：`SimpleActionServer` 的 execute 回调内长循环会阻塞新 goal 接受。
- **Feedback**：Server 调用 `PublishFeedback`；Client 注册 `feedback_callback`。
- **Autonomy**：Navigator 以 Action Client 调用 Controller，消息定义见 [commsgs](../14_Commsgs/08_nav_planning_msgs.md)。

---

## 5.6 排错

| 现象 | 处理 |
|------|------|
| Client 一直等 Server | 先启 `action_listener`；核对 `kActionName` |
| Goal rejected | Server execute 回调未正确注册 |
| 无 feedback | 注册 `feedback_callback`；Server 须 `PublishFeedback` |
| 超时 | 调大 `kResultTimeout`；检查 Server 是否卡死 |

---

**导航**：[← §4 Service](04_service.md) · [§0 指南](00_guide.md) · [§6 Parameter →](06_parameter.md)
