# Autolink 文档

文档模块，包含 autolink 框架的详细文档和说明。

## 内容

- API 文档
- 使用指南
- 架构设计文档
- 示例代码

## 使用说明

此目录包含 autolink 框架的完整文档，帮助开发者理解和使用框架的各项功能。

---

# Action 系统使用指南

## 概述

Action 系统是 autolink 框架中用于实现长时间运行任务的通信机制。它类似于 ROS2 的 Action 系统，提供了目标（Goal）、反馈（Feedback）和结果（Result）的完整生命周期管理。

## 核心概念

### 1. Action 消息结构

Action 消息必须包含三个嵌套类型：
- **Goal**: 客户端发送给服务器的目标请求
- **Feedback**: 服务器在执行过程中发送给客户端的进度反馈
- **Result**: 服务器完成任务后返回的最终结果

### 2. Action 状态

- `UNKNOWN`: 未知状态
- `ACCEPTED`: 目标已被接受
- `EXECUTING`: 正在执行
- `CANCELING`: 正在取消
- `SUCCEEDED`: 成功完成
- `CANCELED`: 已取消
- `ABORTED`: 已中止

### 3. Goal Response

- `ACCEPT_AND_EXECUTE`: 接受并立即执行
- `ACCEPT_AND_DEFER`: 接受但延迟执行
- `REJECT`: 拒绝目标

### 4. Cancel Response

- `ACCEPT`: 接受取消请求
- `REJECT`: 拒绝取消请求

## 定义 Action 消息

首先，需要在 proto 文件中定义 Action 消息。例如：

```protobuf
syntax = "proto3";

package examples.proto;

message ExampleAction {
    message Goal {
        int32 target_number = 1;
        string task_name = 2;
    }
    
    message Feedback {
        int32 current_progress = 1;
        double percentage = 2;
        string status_message = 3;
    }
    
    message Result {
        bool success = 1;
        int32 final_count = 2;
        string message = 3;
        int64 execution_time_ms = 4;
    }
    
    Goal goal = 1;
    Feedback feedback = 2;
    Result result = 3;
}
```

## 创建 Action Server

### 1. 基本步骤

```cpp
#include "autolink/action/action.hpp"
#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"

// 定义 Action Wrapper
struct ExampleActionWrapper {
    using Goal = ExampleAction::Goal;
    using Feedback = ExampleAction::Feedback;
    using Result = ExampleAction::Result;
};

int main(int argc, char* argv[]) {
    // 初始化 autolink
    autolink::Init(argv[0]);
    
    // 创建节点
    auto node = autolink::CreateNode("action_server");
    
    // 创建 Action Server
    auto action_server = autolink::action::CreateServer<ExampleActionWrapper>(
        node,
        "example_action",
        // Handle goal callback: 决定是否接受目标
        [](const autolink::action::GoalUUID& uuid, 
           std::shared_ptr<const ExampleAction::Goal> goal) -> GoalResponse {
            // 验证目标
            if (goal->target_number() > 0 && goal->target_number() <= 100) {
                return GoalResponse::ACCEPT_AND_EXECUTE;
            } else {
                return GoalResponse::REJECT;
            }
        },
        // Handle cancel callback: 决定是否接受取消请求
        [](std::shared_ptr<autolink::action::ServerGoalHandle<ExampleActionWrapper>> goal_handle) -> CancelResponse {
            return CancelResponse::ACCEPT;
        },
        // Handle accepted callback: 处理被接受的目标
        [](std::shared_ptr<autolink::action::ServerGoalHandle<ExampleActionWrapper>> goal_handle) {
            // 在独立线程中执行任务
            std::thread([goal_handle]() {
                auto goal = goal_handle->get_goal();
                int32_t target = goal->target_number();
                
                // 执行任务
                for (int32_t i = 0; i < target; ++i) {
                    // 检查是否被取消
                    if (goal_handle->is_canceling()) {
                        auto result = std::make_shared<ExampleAction::Result>();
                        result->set_success(false);
                        result->set_message("Action was canceled");
                        goal_handle->canceled(result);
                        return;
                    }
                    
                    // 发布反馈
                    auto feedback = std::make_shared<ExampleAction::Feedback>();
                    feedback->set_current_progress(i + 1);
                    feedback->set_percentage(static_cast<double>(i + 1) / target);
                    goal_handle->publish_feedback(feedback);
                    
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                
                // 完成任务
                auto result = std::make_shared<ExampleAction::Result>();
                result->set_success(true);
                result->set_final_count(target);
                result->set_message("Action completed successfully");
                goal_handle->succeed(result);
            }).detach();
        }
    );
    
    // 保持服务器运行
    autolink::WaitForShutdown();
    return 0;
}
```

### 2. Server Goal Handle 方法

- `get_goal()`: 获取目标
- `get_goal_id()`: 获取目标 ID
- `is_active()`: 检查目标是否仍处于活动状态
- `is_executing()`: 检查目标是否正在执行
- `is_canceling()`: 检查目标是否正在被取消
- `publish_feedback(feedback)`: 发布反馈
- `succeed(result)`: 标记任务成功完成
- `canceled(result)`: 标记任务已取消
- `abort(result)`: 标记任务已中止

## 创建 Action Client

### 1. 基本步骤

```cpp
#include "autolink/action/action.hpp"
#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"

// 定义 Action Wrapper
struct ExampleActionWrapper {
    using Goal = ExampleAction::Goal;
    using Feedback = ExampleAction::Feedback;
    using Result = ExampleAction::Result;
};

int main(int argc, char* argv[]) {
    // 初始化 autolink
    autolink::Init(argv[0]);
    
    // 创建节点
    auto node = autolink::CreateNode("action_client");
    
    // 创建 Action Client
    auto action_client = autolink::action::CreateClient<ExampleActionWrapper>(
        node,
        "example_action"
    );
    
    // 等待服务器就绪
    while (!action_client->action_server_is_ready()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 创建目标
    ExampleAction::Goal goal;
    goal.set_target_number(10);
    goal.set_task_name("Count to 10");
    
    // 配置发送选项
    autolink::action::Client<ExampleActionWrapper>::SendGoalOptions options;
    
    // 目标响应回调
    options.goal_response_callback = [](auto goal_handle) {
        if (goal_handle) {
            AINFO << "Goal accepted!";
        } else {
            AWARN << "Goal was rejected!";
        }
    };
    
    // 反馈回调
    options.feedback_callback = [](auto goal_handle, auto feedback) {
        AINFO << "Progress: " << feedback->current_progress() 
              << " (" << feedback->percentage() * 100 << "%)";
    };
    
    // 结果回调
    options.result_callback = [](const auto& wrapped_result) {
        AINFO << "Result received!";
        AINFO << "  Success: " << wrapped_result.result->success();
        AINFO << "  Status: " << static_cast<int>(wrapped_result.code);
    };
    
    // 发送目标
    auto goal_future = action_client->async_send_goal(goal, options);
    auto goal_handle = goal_future.get();
    
    if (!goal_handle) {
        AERROR << "Failed to send goal!";
        return 1;
    }
    
    // 获取结果
    auto result_future = action_client->async_get_result(goal_handle);
    auto wrapped_result = result_future.get();
    
    AINFO << "Final result: " << wrapped_result.result->message();
    
    return 0;
}
```

### 2. Client Goal Handle 方法

- `get_goal_id()`: 获取目标 ID
- `get_status()`: 获取当前状态
- `async_get_result()`: 异步获取结果

### 3. 取消目标

```cpp
// 取消目标
auto cancel_future = action_client->async_cancel_goal(goal_handle);
bool canceled = cancel_future.get();

if (canceled) {
    AINFO << "Cancel request accepted!";
} else {
    AWARN << "Cancel request rejected!";
}
```

## 完整示例

### 运行示例

1. **启动 Action Server**:
```bash
export AUTOLINK_PATH=/path/to/autolink/autolink
./autolink.examples.action_server
```

2. **运行 Action Client**:
```bash
export AUTOLINK_PATH=/path/to/autolink/autolink
# 基本使用
./autolink.examples.action_client 10

# 带取消功能（10秒后取消）
./autolink.examples.action_client 20 1
```

3. **运行测试套件**:
```bash
export AUTOLINK_PATH=/path/to/autolink/autolink
./autolink.examples.action_test
```

## 测试用例

测试程序 `action_test_example.cpp` 包含以下测试场景：

1. **TestSuccessCase**: 基本成功场景测试
2. **TestRejectionCase**: 目标拒绝场景测试
3. **TestCancelCase**: 取消场景测试
4. **TestBoundaryCases**: 边界值测试（最小值、最大值、无效值）
5. **TestConcurrentGoals**: 并发目标测试

## 最佳实践

### 1. 错误处理

- 始终检查 `goal_handle` 是否为 `nullptr`
- 使用 try-catch 捕获异常
- 检查服务器是否就绪

### 2. 反馈发布

- 定期发布反馈（建议每 10% 或关键里程碑）
- 避免过于频繁的反馈发布
- 在反馈中包含有意义的进度信息

### 3. 取消处理

- 在执行循环中定期检查 `is_canceling()`
- 及时响应取消请求
- 清理资源并返回适当的结果

### 4. 线程安全

- 在独立线程中执行长时间任务
- 使用适当的同步机制
- 避免阻塞主线程

### 5. 资源管理

- 使用智能指针管理资源
- 及时清理不再需要的 goal handles
- 正确处理异常情况

## API 参考

### Action Server

- `autolink::action::CreateServer<ActionT>(node, action_name, handle_goal, handle_cancel, handle_accepted)`: 创建 Action Server

### Action Client

- `autolink::action::CreateClient<ActionT>(node, action_name)`: 创建 Action Client
- `action_client->async_send_goal(goal, options)`: 异步发送目标
- `action_client->async_cancel_goal(goal_handle)`: 异步取消目标
- `action_client->async_get_result(goal_handle)`: 异步获取结果
- `action_client->action_server_is_ready()`: 检查服务器是否就绪

### 类型定义

- `autolink::action::GoalUUID`: 目标 UUID（16 字节数组）
- `autolink::action::GoalStatus`: 目标状态枚举
- `autolink::action::GoalResponse`: 目标响应枚举
- `autolink::action::CancelResponse`: 取消响应枚举
- `autolink::action::ResultCode`: 结果代码枚举

## 注意事项

1. **环境变量**: 必须设置 `AUTOLINK_PATH` 环境变量指向 autolink 配置目录
2. **消息定义**: Action 消息必须包含 Goal、Feedback、Result 三个嵌套类型
3. **线程安全**: Server 和 Client 都是线程安全的，可以在多线程环境中使用
4. **生命周期**: Goal Handle 的生命周期由智能指针管理，无需手动释放

## 更多信息

- 示例代码: `src/autonomy/autolink/autolink/examples/action_server.cpp`
- 示例代码: `src/autonomy/autolink/autolink/examples/action_client.cpp`
- 测试代码: `src/autonomy/autolink/autolink/examples/action_test_example.cpp`
