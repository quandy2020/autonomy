# 5. 动作 Action

长时任务：Goal / Feedback / Result，支持取消与抢占。类似 ROS 2 Action。

## 5.1 流程

```text
Client AsyncSendGoal → Server 接受 → 周期 Feedback → Result / Cancel
```

消息需 Goal / Feedback / Result 三部分（见 `examples` proto）。Traits 绑定类型后交给 `CreateServer` / `CreateClient`。

## 5.2 Server

```cpp
constexpr char kActionName[] = "examples/simple_message_action";
auto server = autolink::action::CreateServer<Traits>(
    node, kActionName, handle_goal, handle_cancel, handle_accepted);
```

`handle_accepted` 内跑目标逻辑，周期发 Feedback，结束时 `Succeeded` / `Aborted` / `Canceled`。源码：`action_listener.cpp`。

## 5.3 Client

```cpp
auto client = autolink::action::CreateClient<Traits>(node, kActionName);
client->AsyncSendGoal(goal, options); // feedback / result 回调
```

源码：`action_talker.cpp`。

## 5.4 运行

```bash
./action_listener # terminal 1
./action_talker # terminal 2
```

## 5.5 集成

| 场景 | 用法 |
|------|------|
| 导航长任务 | Task → Planning / Control Action |
| 可取消 | Client 发 Cancel；Server `handle_cancel` |
