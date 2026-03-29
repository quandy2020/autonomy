/**
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <functional>
#include <memory>
#include <mutex>

#include "autolink/action/types.hpp"

namespace autolink {
namespace action {

// Forward declaration
template <typename ActionT>
class Server;

// Forward declaration for friend
template <typename ActionT>
class ServerGoalHandle;

/**
 * @class ServerGoalHandle
 * @brief Class to interact with goals on a server.
 *
 * Use this class to check the status of a goal as well as set the result.
 * This class is not meant to be created by a user, instead it is created
 * when a goal has been accepted.
 *
 * @tparam ActionT The action type, which should have Goal, Feedback, and
 * Result types defined.
 */
template <typename ActionT>
class ServerGoalHandle
{
public:
    using Goal = typename ActionT::Goal;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;

    /**
     * @brief Get the user provided message describing the goal.
     *
     * @return const std::shared_ptr<const Goal> The goal message.
     */
    const std::shared_ptr<const Goal> GetGoal() const {
        return goal_;
    }

    /**
     * @brief Get the unique identifier of the goal.
     *
     * @return const GoalUUID& The goal UUID.
     */
    const GoalUUID& GetGoalId() const {
        return uuid_;
    }

    /**
     * @brief Check if client has requested this goal be cancelled.
     *
     * @return true if a cancellation request has been accepted for this goal.
     */
    bool IsCanceling() const;

    /**
     * @brief Transition ACCEPTED or EXECUTING to CANCELING after cancel is
     * accepted. Called by Server::HandleCancelGoal; not for application use.
     *
     * @return true if the goal was in a cancelable state.
     */
    bool NotifyCancelRequestAccepted();

    /**
     * @brief Check if goal is pending or executing.
     *
     * @return false if goal has reached a terminal state.
     */
    bool IsActive() const;

    /**
     * @brief Check if goal is executing.
     *
     * @return true only if the goal is in an executing state.
     */
    bool IsExecuting() const;

    /**
     * @brief Send an update about the progress of a goal.
     *
     * This must only be called when the goal is executing.
     *
     * @param feedback_msg The feedback message to publish to clients.
     */
    void PublishFeedback(std::shared_ptr<Feedback> feedback_msg);

    /**
     * @brief Get the current status of the goal.
     *
     * @return GoalStatus The current status.
     */
    GoalStatus GetStatus() const;

    /** Same encoding as ROS 2 action_msgs/GoalStatus (0..6). */
    int8_t GetStatusCode() const {
        return static_cast<int8_t>(GetStatus());
    }

    bool IsSucceeded() const;
    bool IsAborted() const;
    bool IsCanceled() const;

    /**
     * @brief Indicate that a goal could not be reached and has been aborted.
     *
     * Only call this if the goal was executing but cannot be completed.
     * This is a terminal state, no more methods should be called on a goal
     * handle after this is called.
     *
     * @param result_msg The final result to send to clients.
     */
    void Abort(std::shared_ptr<Result> result_msg);

    /**
     * @brief Indicate that a goal has succeeded.
     *
     * Only call this if the goal is executing and has reached the desired final
     * state. This is a terminal state, no more methods should be called on a
     * goal handle after this is called.
     *
     * @param result_msg The final result to send to clients.
     */
    void Succeed(std::shared_ptr<Result> result_msg);

    /**
     * @brief Indicate that a goal has been canceled.
     *
     * Only call this if the goal is canceling.
     * This is a terminal state, no more methods should be called on a goal
     * handle after this is called.
     *
     * @param result_msg The final result to send to clients.
     */
    void Canceled(std::shared_ptr<Result> result_msg);

    /**
     * @brief Indicate that the server is starting to execute a goal.
     *
     * Only call this if the goal is pending.
     */
    void Execute();

    virtual ~ServerGoalHandle();

    /**
     * @brief Construct a new ServerGoalHandle object.
     *
     * @note This constructor is public but should only be called by Server.
     * Users should not create ServerGoalHandle instances directly.
     *
     * @param uuid The unique identifier for the goal.
     * @param goal The goal message.
     * @param on_terminal_state Callback to call when goal reaches terminal
     * state.
     * @param on_executing Callback to call when goal starts executing.
     * @param publish_feedback Callback to publish feedback.
     */
    ServerGoalHandle(
        const GoalUUID& uuid, std::shared_ptr<const Goal> goal,
        std::function<void(const GoalUUID&, std::shared_ptr<Result>)>
            on_terminal_state,
        std::function<void(const GoalUUID&)> on_executing,
        std::function<void(std::shared_ptr<Feedback>)> publish_feedback);

private:
    GoalUUID uuid_;
    std::shared_ptr<const Goal> goal_;
    mutable std::mutex status_mutex_;
    GoalStatus status_;

    std::function<void(const GoalUUID&, std::shared_ptr<Result>)>
        on_terminal_state_;
    std::function<void(const GoalUUID&)> on_executing_;
    std::function<void(std::shared_ptr<Feedback>)> publish_feedback_;
};

template <typename ActionT>
ServerGoalHandle<ActionT>::ServerGoalHandle(
    const GoalUUID& uuid, std::shared_ptr<const Goal> goal,
    std::function<void(const GoalUUID&,
                       std::shared_ptr<typename ActionT::Result>)>
        on_terminal_state,
    std::function<void(const GoalUUID&)> on_executing,
    std::function<void(std::shared_ptr<typename ActionT::Feedback>)>
        publish_feedback)
    : uuid_(uuid),
      goal_(goal),
      status_(GoalStatus::ACCEPTED),
      on_terminal_state_(on_terminal_state),
      on_executing_(on_executing),
      publish_feedback_(publish_feedback) {}

template <typename ActionT>
ServerGoalHandle<ActionT>::~ServerGoalHandle() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    // If goal hasn't reached terminal state, abort it
    if (status_ != GoalStatus::SUCCEEDED && status_ != GoalStatus::CANCELED &&
        status_ != GoalStatus::ABORTED) {
        auto null_result = std::make_shared<typename ActionT::Result>();
        on_terminal_state_(uuid_, null_result);
    }
}

template <typename ActionT>
bool ServerGoalHandle<ActionT>::IsCanceling() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_ == GoalStatus::CANCELING;
}

template <typename ActionT>
bool ServerGoalHandle<ActionT>::NotifyCancelRequestAccepted() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (status_ == GoalStatus::ACCEPTED || status_ == GoalStatus::EXECUTING) {
        status_ = GoalStatus::CANCELING;
        return true;
    }
    return false;
}

template <typename ActionT>
bool ServerGoalHandle<ActionT>::IsActive() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_ == GoalStatus::ACCEPTED ||
           status_ == GoalStatus::EXECUTING || status_ == GoalStatus::CANCELING;
}

template <typename ActionT>
bool ServerGoalHandle<ActionT>::IsExecuting() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_ == GoalStatus::EXECUTING;
}

template <typename ActionT>
void ServerGoalHandle<ActionT>::PublishFeedback(
    std::shared_ptr<typename ActionT::Feedback> feedback_msg) {
    if (!feedback_msg) {
        AWARN << "PublishFeedback: received null feedback message";
        return;
    }

    if (!IsExecuting()) {
        GoalStatus current_status = GetStatus();
        AWARN << "PublishFeedback: goal is not executing. Current status: "
              << static_cast<int>(current_status)
              << ". Goal ID: " << ToString(uuid_);
        return;
    }

    if (publish_feedback_) {
        ADEBUG << "ServerGoalHandle::PublishFeedback: calling "
                  "publish_feedback_ callback for goal ID: "
               << ToString(uuid_);
        publish_feedback_(feedback_msg);
    } else {
        AWARN << "ServerGoalHandle::PublishFeedback: publish_feedback_ "
                 "callback is null for goal ID: "
              << ToString(uuid_);
    }
}

template <typename ActionT>
GoalStatus ServerGoalHandle<ActionT>::GetStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

template <typename ActionT>
bool ServerGoalHandle<ActionT>::IsSucceeded() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_ == GoalStatus::SUCCEEDED;
}

template <typename ActionT>
bool ServerGoalHandle<ActionT>::IsAborted() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_ == GoalStatus::ABORTED;
}

template <typename ActionT>
bool ServerGoalHandle<ActionT>::IsCanceled() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_ == GoalStatus::CANCELED;
}

template <typename ActionT>
void ServerGoalHandle<ActionT>::Execute() {
    bool should_call_callback = false;
    {
        std::unique_lock<std::mutex> lock(status_mutex_);
        if (status_ == GoalStatus::ACCEPTED) {
            status_ = GoalStatus::EXECUTING;
            ADEBUG << "ServerGoalHandle::Execute: status changed to EXECUTING "
                      "for goal ID: "
                   << ToString(uuid_);
            should_call_callback = (on_executing_ != nullptr);
        } else {
            AWARN << "ServerGoalHandle::Execute: status is not ACCEPTED, "
                     "current status="
                  << static_cast<int>(status_)
                  << " for goal ID: " << ToString(uuid_);
        }
    }
    // Release lock before calling callback to avoid deadlock
    // (callback may call GetStatus() which needs the same lock)
    if (should_call_callback) {
        ADEBUG << "ServerGoalHandle::Execute: calling on_executing_ callback "
                  "for goal ID: "
               << ToString(uuid_);
        on_executing_(uuid_);
    } else if (on_executing_ == nullptr) {
        AWARN << "ServerGoalHandle::Execute: on_executing_ callback is null "
                 "for goal ID: "
              << ToString(uuid_);
    }
}

template <typename ActionT>
void ServerGoalHandle<ActionT>::Abort(
    std::shared_ptr<typename ActionT::Result> result_msg) {
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (status_ != GoalStatus::EXECUTING &&
            status_ != GoalStatus::CANCELING) {
            return;
        }
        status_ = GoalStatus::ABORTED;
    }
    if (on_terminal_state_) {
        on_terminal_state_(uuid_, result_msg);
    }
}

template <typename ActionT>
void ServerGoalHandle<ActionT>::Succeed(
    std::shared_ptr<typename ActionT::Result> result_msg) {
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (status_ != GoalStatus::EXECUTING) {
            return;
        }
        status_ = GoalStatus::SUCCEEDED;
    }
    if (on_terminal_state_) {
        on_terminal_state_(uuid_, result_msg);
    }
}

template <typename ActionT>
void ServerGoalHandle<ActionT>::Canceled(
    std::shared_ptr<typename ActionT::Result> result_msg) {
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (status_ != GoalStatus::CANCELING) {
            return;
        }
        status_ = GoalStatus::CANCELED;
    }
    if (on_terminal_state_) {
        on_terminal_state_(uuid_, result_msg);
    }
}

}  // namespace action
}  // namespace autolink
