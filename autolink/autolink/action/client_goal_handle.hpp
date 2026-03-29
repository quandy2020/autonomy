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
#include <future>
#include <memory>
#include <mutex>

#include "autolink/action/exceptions.hpp"
#include "autolink/action/types.hpp"
#include "autolink/common/log.hpp"

namespace autolink {
namespace action {

// Forward declaration
template <typename ActionT>
class Client;

/**
 * @class ClientGoalHandle
 * @brief Class for interacting with goals sent from action clients.
 *
 * Use this class to check the status of a goal as well as get the result.
 * This class is not meant to be created by a user, instead it is created
 * when a goal has been accepted.
 *
 * @tparam ActionT The action type, which should have Goal, Feedback, and
 * Result types defined.
 */
template <typename ActionT>
class ClientGoalHandle
{
public:
    using Goal = typename ActionT::Goal;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;

    /**
     * @brief A wrapper that defines the result of an action.
     */
    struct WrappedResult {
        /// The unique identifier of the goal
        GoalUUID goal_id;
        /// A status to indicate if the goal was canceled, aborted, or succeeded
        ResultCode code;
        /// User defined fields sent back with an action
        std::shared_ptr<Result> result;
    };

    using FeedbackCallback =
        std::function<void(std::shared_ptr<ClientGoalHandle<ActionT>>,
                           std::shared_ptr<const Feedback>)>;
    using ResultCallback = std::function<void(const WrappedResult&)>;

    /**
     * @brief Get the unique ID for the goal.
     *
     * @return const GoalUUID& The goal UUID.
     */
    const GoalUUID& GetGoalId() const {
        return goal_info_.goal_id;
    }

    /**
     * @brief Get the time when the goal was accepted.
     *
     * @return int64_t Timestamp in nanoseconds.
     */
    int64_t GetGoalStamp() const {
        return goal_info_.stamp;
    }

    /**
     * @brief Status as int8_t; same encoding as ROS 2 action_msgs/GoalStatus
     * (see https://design.ros2.org/articles/actions.html): 0=unknown,
     * 1=accepted, 2=executing, 3=canceling, 4=succeeded, 5=canceled, 6=aborted.
     */
    int8_t GetStatus() const;

    /** Typed goal status (same values as GetStatus()). */
    GoalStatus GetGoalStatus() const;

    /** Terminal: status == succeeded (4). */
    bool IsSucceeded() const;

    /** Terminal: status == aborted (6). */
    bool IsAborted() const;

    /** Terminal: status == canceled (5). */
    bool IsCanceled() const;

    /**
     * @brief Check if an action client has subscribed to feedback for the goal.
     *
     * @return true if feedback callback is set.
     */
    bool IsFeedbackAware() const;

    /**
     * @brief Check if an action client has requested the result for the goal.
     *
     * @return true if result awareness is set.
     */
    bool IsResultAware() const;

    /**
     * @brief Get a future to the goal result.
     *
     * @return std::shared_future<WrappedResult> A future to the result.
     * @throws UnawareGoalHandleError If the goal handle is unaware of the
     * result.
     */
    std::shared_future<WrappedResult> AsyncGetResult();

    virtual ~ClientGoalHandle();

    /**
     * @brief Construct a new ClientGoalHandle object.
     *
     * @note This constructor is public but should only be called by Client.
     * Users should not create ClientGoalHandle instances directly.
     *
     * @param goal_info The goal information.
     * @param feedback_callback Callback for feedback messages.
     * @param result_callback Callback for result messages.
     */
    ClientGoalHandle(const GoalInfo& goal_info,
                     FeedbackCallback feedback_callback,
                     ResultCallback result_callback);

    // Client needs access to these private methods
    template <typename T>
    friend class Client;

private:
    void SetFeedbackCallback(FeedbackCallback callback);
    void SetResultCallback(ResultCallback callback);
    void CallFeedbackCallback(
        std::shared_ptr<ClientGoalHandle<ActionT>> shared_this,
        std::shared_ptr<const Feedback> feedback_message);

    /// Returns the previous value of awareness
    bool SetResultAwareness(bool awareness);

    void SetStatus(int8_t status);
    void SetResult(const WrappedResult& wrapped_result);
    void Invalidate(const UnawareGoalHandleError& ex);
    bool IsInvalidated() const;

    GoalInfo goal_info_;
    std::exception_ptr invalidate_exception_{nullptr};

    bool is_result_aware_{false};
    std::promise<WrappedResult> result_promise_;
    std::shared_future<WrappedResult> result_future_;

    FeedbackCallback feedback_callback_{nullptr};
    ResultCallback result_callback_{nullptr};
    int8_t status_{static_cast<int8_t>(GoalStatus::ACCEPTED)};

    mutable std::recursive_mutex handle_mutex_;
};

template <typename ActionT>
ClientGoalHandle<ActionT>::ClientGoalHandle(const GoalInfo& goal_info,
                                            FeedbackCallback feedback_callback,
                                            ResultCallback result_callback)
    : goal_info_(goal_info),
      result_future_(result_promise_.get_future()),
      feedback_callback_(feedback_callback),
      result_callback_(result_callback) {}

template <typename ActionT>
ClientGoalHandle<ActionT>::~ClientGoalHandle() {
    // If result hasn't been set, invalidate the promise
    if (!is_result_aware_) {
        try {
            WrappedResult error_result;
            error_result.goal_id = goal_info_.goal_id;
            error_result.code = ResultCode::ABORTED;
            error_result.result = std::make_shared<Result>();
            status_ = static_cast<int8_t>(GoalStatus::ABORTED);
            result_promise_.set_value(error_result);
        } catch (...) {
            // Ignore exceptions in destructor
        }
    }
}

template <typename ActionT>
int8_t ClientGoalHandle<ActionT>::GetStatus() const {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    return status_;
}

template <typename ActionT>
GoalStatus ClientGoalHandle<ActionT>::GetGoalStatus() const {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    return static_cast<GoalStatus>(status_);
}

template <typename ActionT>
bool ClientGoalHandle<ActionT>::IsSucceeded() const {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    return status_ == static_cast<int8_t>(GoalStatus::SUCCEEDED);
}

template <typename ActionT>
bool ClientGoalHandle<ActionT>::IsAborted() const {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    return status_ == static_cast<int8_t>(GoalStatus::ABORTED);
}

template <typename ActionT>
bool ClientGoalHandle<ActionT>::IsCanceled() const {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    return status_ == static_cast<int8_t>(GoalStatus::CANCELED);
}

template <typename ActionT>
bool ClientGoalHandle<ActionT>::IsFeedbackAware() const {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    return feedback_callback_ != nullptr;
}

template <typename ActionT>
bool ClientGoalHandle<ActionT>::IsResultAware() const {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    return is_result_aware_;
}

template <typename ActionT>
std::shared_future<typename ClientGoalHandle<ActionT>::WrappedResult>
ClientGoalHandle<ActionT>::AsyncGetResult() {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    if (this->IsInvalidated()) {
        std::rethrow_exception(invalidate_exception_);
    }
    if (!is_result_aware_) {
        throw UnawareGoalHandleError("Goal handle is not aware of the result");
    }
    return result_future_;
}

template <typename ActionT>
void ClientGoalHandle<ActionT>::SetFeedbackCallback(FeedbackCallback callback) {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    feedback_callback_ = callback;
}

template <typename ActionT>
void ClientGoalHandle<ActionT>::SetResultCallback(ResultCallback callback) {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    result_callback_ = callback;
}

template <typename ActionT>
void ClientGoalHandle<ActionT>::CallFeedbackCallback(
    std::shared_ptr<ClientGoalHandle<ActionT>> shared_this,
    std::shared_ptr<const Feedback> feedback_message) {
    FeedbackCallback callback;
    {
        std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
        callback = feedback_callback_;
    }
    if (callback) {
        ADEBUG << "CallFeedbackCallback: calling user feedback callback";
        callback(shared_this, feedback_message);
    } else {
        ADEBUG << "CallFeedbackCallback: feedback callback is null";
    }
}

template <typename ActionT>
bool ClientGoalHandle<ActionT>::SetResultAwareness(bool awareness) {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    bool previous = is_result_aware_;
    is_result_aware_ = awareness;
    return previous;
}

template <typename ActionT>
void ClientGoalHandle<ActionT>::SetStatus(int8_t status) {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    status_ = status;
}

template <typename ActionT>
void ClientGoalHandle<ActionT>::SetResult(const WrappedResult& wrapped_result) {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    // Align with ROS 2 terminal status codes (ResultCode matches terminal
    // GoalStatus).
    status_ = static_cast<int8_t>(wrapped_result.code);
    result_promise_.set_value(wrapped_result);
    if (result_callback_) {
        result_callback_(wrapped_result);
    }
}

template <typename ActionT>
void ClientGoalHandle<ActionT>::Invalidate(const UnawareGoalHandleError& ex) {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    invalidate_exception_ = std::make_exception_ptr(ex);
}

template <typename ActionT>
bool ClientGoalHandle<ActionT>::IsInvalidated() const {
    std::lock_guard<std::recursive_mutex> lock(handle_mutex_);
    return invalidate_exception_ != nullptr;
}

}  // namespace action
}  // namespace autolink
