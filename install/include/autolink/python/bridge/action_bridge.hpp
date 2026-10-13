/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "autolink/action/types.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/node/node.hpp"

namespace autolink {
namespace python_support {

struct BytesActionTraits {
    using Goal = message::RawMessage;
    using Feedback = message::RawMessage;
    using Result = message::RawMessage;
};

class ClientGoalHandleView {
public:
    using FeedbackCallback =
        std::function<void(const std::shared_ptr<ClientGoalHandleView>&,
                           const std::string& feedback_bytes)>;
    using ResultCallback =
        std::function<void(const std::string& result_bytes, int8_t result_code)>;

    virtual ~ClientGoalHandleView() = default;

    virtual std::string GoalIdHex() const = 0;
    virtual int8_t Status() const = 0;
    virtual bool IsSucceeded() const = 0;
    virtual bool IsCanceled() const = 0;
    virtual bool IsAborted() const = 0;

    // Blocks until result; returns empty string on failure.
    virtual std::string GetResult(double timeout_sec) = 0;
    virtual int8_t GetResultCode() const = 0;
    virtual bool Cancel(double timeout_sec) = 0;
};

class ActionClientHandle {
public:
    using GoalResponseCallback =
        std::function<void(const std::shared_ptr<ClientGoalHandleView>&)>;
    using FeedbackCallback = ClientGoalHandleView::FeedbackCallback;
    using ResultCallback = ClientGoalHandleView::ResultCallback;

    ActionClientHandle(const std::shared_ptr<Node>& node,
                       const std::string& action_name);
    ~ActionClientHandle();

    ActionClientHandle(const ActionClientHandle&) = delete;
    ActionClientHandle& operator=(const ActionClientHandle&) = delete;

    bool ServerIsReady() const;
    bool WaitForServer(double timeout_sec);

    // Async send; callbacks may run on action threads.
    std::shared_ptr<ClientGoalHandleView> SendGoalAsync(
        const std::string& goal_bytes, GoalResponseCallback goal_response_cb,
        FeedbackCallback feedback_cb, ResultCallback result_cb);

    // Sync: wait until accepted (or timeout). Returns nullptr if rejected/timeout.
    std::shared_ptr<ClientGoalHandleView> SendGoal(
        const std::string& goal_bytes, FeedbackCallback feedback_cb,
        double timeout_sec);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

class SimpleActionServerHandle {
public:
    using ExecuteCallback = std::function<void()>;
    using CompletionCallback = std::function<void()>;

    SimpleActionServerHandle(const std::shared_ptr<Node>& node,
                             const std::string& action_name,
                             ExecuteCallback execute_cb,
                             CompletionCallback completion_cb = nullptr);
    ~SimpleActionServerHandle();

    SimpleActionServerHandle(const SimpleActionServerHandle&) = delete;
    SimpleActionServerHandle& operator=(const SimpleActionServerHandle&) =
        delete;

    std::string GetCurrentGoal() const;
    bool IsCancelRequested() const;
    bool IsPreemptRequested() const;
    bool AcceptPendingGoal();
    void PublishFeedback(const std::string& feedback_bytes);
    void SucceededCurrent(const std::string& result_bytes);
    void TerminateCurrent(const std::string& result_bytes);
    bool HasFeedbackSubscriber() const;
    void Activate();
    void Deactivate();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

class ServerGoalHandleView {
public:
    virtual ~ServerGoalHandleView() = default;
    virtual std::string GetGoal() const = 0;
    virtual std::string GoalIdHex() const = 0;
    virtual void PublishFeedback(const std::string& feedback_bytes) = 0;
    virtual void Succeed(const std::string& result_bytes) = 0;
    virtual void Abort(const std::string& result_bytes) = 0;
    virtual void Canceled(const std::string& result_bytes) = 0;
    virtual void Execute() = 0;
    virtual bool IsActive() const = 0;
    virtual bool IsExecuting() const = 0;
    virtual bool IsCanceling() const = 0;
    virtual int8_t GetStatus() const = 0;
};

class ActionServerHandle {
public:
    using GoalCallback = std::function<action::GoalResponse(
        const std::string& goal_id_hex, const std::string& goal_bytes)>;
    using CancelCallback =
        std::function<action::CancelResponse(
            const std::shared_ptr<ServerGoalHandleView>&)>;
    using AcceptedCallback =
        std::function<void(const std::shared_ptr<ServerGoalHandleView>&)>;

    ActionServerHandle(const std::shared_ptr<Node>& node,
                       const std::string& action_name, GoalCallback handle_goal,
                       CancelCallback handle_cancel,
                       AcceptedCallback handle_accepted);
    ~ActionServerHandle();

    ActionServerHandle(const ActionServerHandle&) = delete;
    ActionServerHandle& operator=(const ActionServerHandle&) = delete;

    bool HasFeedbackSubscriber() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

std::string GoalUUIDToHex(const action::GoalUUID& uuid);
action::GoalUUID GoalUUIDFromHex(const std::string& hex);

}  // namespace python_support
}  // namespace autolink
