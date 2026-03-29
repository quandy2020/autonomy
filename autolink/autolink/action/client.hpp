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

#include <chrono>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "autolink/action/client_goal_handle.hpp"
#include "autolink/action/exceptions.hpp"
#include "autolink/action/internal_types.hpp"
#include "autolink/action/types.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/macros.hpp"
#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"
#include "autolink/service/client.hpp"

namespace autolink {
namespace action {

// Forward declaration
template <typename ActionT>
class Client;

/**
 * @class Client
 * @brief Action client implementation.
 *
 * This class creates an action client that can send goals, cancel goals,
 * and receive feedback and results. It uses autolink's Service and Channel
 * mechanisms.
 *
 * @tparam ActionT The action type, which should have Goal, Feedback, and
 * Result types defined.
 */
template <typename ActionT>
class Client
{
public:
    using Goal = typename ActionT::Goal;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;
    using GoalHandle = ClientGoalHandle<ActionT>;
    using WrappedResult = typename GoalHandle::WrappedResult;

    /**
     * @brief Defines shared_ptr and weak_ptr for Client.
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(Client)

    /// Options for sending a goal.
    struct SendGoalOptions {
        /// Function called when the goal is accepted or rejected.
        std::function<void(std::shared_ptr<GoalHandle>)> goal_response_callback;

        /// Function called whenever feedback is received for the goal.
        typename GoalHandle::FeedbackCallback feedback_callback;

        /// Function called when the result for the goal is received.
        typename GoalHandle::ResultCallback result_callback;
    };

    /**
     * @brief Construct a new Client object.
     *
     * @param node The node to create the client on.
     * @param action_name The name of the action.
     */
    explicit Client(std::shared_ptr<Node> node, const std::string& action_name)
        : node_(node), action_name_(action_name) {
        Init();
    }

    virtual ~Client() {
        std::lock_guard<std::recursive_mutex> guard(goal_handles_mutex_);
        for (auto& [goal_id, weak_handle] : goal_handles_) {
            auto goal_handle = weak_handle.lock();
            if (goal_handle) {
                goal_handle->Invalidate(UnawareGoalHandleError());
            }
        }
        goal_handles_.clear();
    }

    /**
     * @brief Send an action goal and asynchronously get the result.
     *
     * @param goal The goal request.
     * @param options Options for sending the goal request.
     * @return std::shared_future<std::shared_ptr<GoalHandle>> A future that
     * completes when the goal has been accepted or rejected.
     */
    std::shared_future<std::shared_ptr<GoalHandle>> AsyncSendGoal(
        const Goal& goal, const SendGoalOptions& options = SendGoalOptions());

    /**
     * @brief Asynchronously get the result for an active goal.
     *
     * @param goal_handle The goal handle for which to get the result.
     * @param result_callback Optional callback that is called when the result
     * is received.
     * @return std::shared_future<WrappedResult> A future that is set to the
     * goal result when the goal is finished.
     * @throws UnknownGoalHandleError If the goal is unknown.
     */
    std::shared_future<WrappedResult> AsyncGetResult(
        std::shared_ptr<GoalHandle> goal_handle,
        typename GoalHandle::ResultCallback result_callback = nullptr);

    /**
     * @brief Asynchronously request a goal be canceled.
     *
     * @param goal_handle The goal handle requesting to be canceled.
     * @return std::shared_future<bool> A future that is set to true if the
     * cancel request was accepted.
     * @throws UnknownGoalHandleError If the goal is unknown.
     */
    std::shared_future<bool> AsyncCancelGoal(
        std::shared_ptr<GoalHandle> goal_handle);

    /**
     * @brief Check if the action server is ready.
     *
     * @return true if the server is ready, false otherwise.
     */
    bool ActionServerIsReady() const;

private:
    void Init();

    void HandleFeedback(
        const std::shared_ptr<internal::FeedbackMessage<ActionT>>&
            feedback_msg);
    void HandleStatus(
        const std::shared_ptr<internal::StatusMessage>& status_msg);

    std::shared_ptr<Node> node_;
    std::string action_name_;

    // Services
    std::shared_ptr<autolink::Client<internal::SendGoalRequest<ActionT>,
                                     internal::SendGoalResponse>>
        send_goal_client_;
    std::shared_ptr<autolink::Client<internal::CancelGoalRequest,
                                     internal::CancelGoalResponse>>
        cancel_goal_client_;
    std::shared_ptr<autolink::Client<internal::GetResultRequest,
                                     internal::GetResultResponse<ActionT>>>
        get_result_client_;

    // Subscribers
    std::shared_ptr<Reader<internal::FeedbackMessage<ActionT>>>
        feedback_reader_;
    std::shared_ptr<Reader<internal::StatusMessage>> status_reader_;

    // Goal handles
    std::map<GoalUUID, std::weak_ptr<GoalHandle>> goal_handles_;
    mutable std::recursive_mutex goal_handles_mutex_;
};

template <typename ActionT>
void Client<ActionT>::Init() {
    // Create SendGoal client
    send_goal_client_ = node_->CreateClient<internal::SendGoalRequest<ActionT>,
                                            internal::SendGoalResponse>(
        action_name_ + "/send_goal");

    // Create CancelGoal client
    cancel_goal_client_ = node_->CreateClient<internal::CancelGoalRequest,
                                              internal::CancelGoalResponse>(
        action_name_ + "/cancel_goal");

    // Create GetResult client
    get_result_client_ =
        node_->CreateClient<internal::GetResultRequest,
                            internal::GetResultResponse<ActionT>>(
            action_name_ + "/get_result");

    // Create feedback reader
    // Use std::bind to avoid lambda type issues in C++11
    auto feedback_callback = std::bind(&Client<ActionT>::HandleFeedback, this,
                                       std::placeholders::_1);
    std::string feedback_channel = action_name_ + "/feedback";
    feedback_reader_ = node_->CreateReader<internal::FeedbackMessage<ActionT>>(
        feedback_channel, feedback_callback);
    if (!feedback_reader_) {
        AERROR << "Client::Init: Failed to create feedback reader";
    }

    // Create status reader
    auto status_callback =
        std::bind(&Client<ActionT>::HandleStatus, this, std::placeholders::_1);
    status_reader_ = node_->CreateReader<internal::StatusMessage>(
        action_name_ + "/status", status_callback);
}

template <typename ActionT>
std::shared_future<std::shared_ptr<typename Client<ActionT>::GoalHandle>>
Client<ActionT>::AsyncSendGoal(const Goal& goal,
                               const SendGoalOptions& options) {
    auto promise =
        std::make_shared<std::promise<std::shared_ptr<GoalHandle>>>();
    std::shared_future<std::shared_ptr<GoalHandle>> future(
        promise->get_future());

    // Generate goal UUID
    GoalUUID goal_id = GenerateGoalUUID();

    // Create request
    auto request = std::make_shared<internal::SendGoalRequest<ActionT>>();
    request->goal_id = goal_id;
    request->goal = goal;

    // Send request
    AINFO << "AsyncSendGoal: sending goal request for goal ID: "
          << ToString(goal_id);
    auto response_future = send_goal_client_->AsyncSendRequest(request);
    AINFO << "AsyncSendGoal: request sent, waiting for response...";
    // Note: std::shared_future doesn't have then() in C++11/14
    // We'll use a thread or callback approach instead
    std::thread([this, promise, goal_id, options, request, response_future]() {
        try {
            AINFO << "AsyncSendGoal: waiting for response for goal ID: "
                  << ToString(goal_id);
            // Wait for response with timeout
            auto status = response_future.wait_for(std::chrono::seconds(10));
            if (status == std::future_status::timeout) {
                AERROR << "AsyncSendGoal: timeout waiting for response for "
                          "goal ID: "
                       << ToString(goal_id);
                promise->set_value(nullptr);
                if (options.goal_response_callback) {
                    options.goal_response_callback(nullptr);
                }
                return;
            }

            auto response = response_future.get();
            AINFO << "AsyncSendGoal: received response for goal ID: "
                  << ToString(goal_id);

            if (!response) {
                AERROR << "AsyncSendGoal: received null response for goal ID: "
                       << ToString(goal_id);
                promise->set_value(nullptr);
                if (options.goal_response_callback) {
                    options.goal_response_callback(nullptr);
                }
                return;
            }

            if (!response->accepted) {
                AWARN << "AsyncSendGoal: goal rejected for goal ID: "
                      << ToString(goal_id);
                promise->set_value(nullptr);
                if (options.goal_response_callback) {
                    options.goal_response_callback(nullptr);
                }
                return;
            }

            ADEBUG << "AsyncSendGoal: goal accepted for goal ID: "
                   << ToString(goal_id);

            // Create goal handle
            GoalInfo goal_info;
            goal_info.goal_id = goal_id;
            goal_info.stamp = response->stamp;

            auto goal_handle = std::make_shared<GoalHandle>(
                goal_info, options.feedback_callback, options.result_callback);

            {
                std::lock_guard<std::recursive_mutex> guard(
                    goal_handles_mutex_);
                goal_handles_[goal_id] = goal_handle;
            }

            ADEBUG << "AsyncSendGoal: created goal handle for goal ID: "
                   << ToString(goal_id);

            // Register get_result polling (sets is_result_aware_) before any
            // waiter receives the handle; otherwise handle->AsyncGetResult()
            // can race and throw UnawareGoalHandleError.
            this->AsyncGetResult(goal_handle, options.result_callback);

            promise->set_value(goal_handle);

            if (options.goal_response_callback) {
                ADEBUG << "AsyncSendGoal: calling goal_response_callback for "
                          "goal ID: "
                       << ToString(goal_id);
                options.goal_response_callback(goal_handle);
            }
        } catch (const std::exception& e) {
            AERROR << "Error in AsyncSendGoal response for goal ID: "
                   << ToString(goal_id) << ": " << e.what();
            promise->set_value(nullptr);
            if (options.goal_response_callback) {
                options.goal_response_callback(nullptr);
            }
        }
    }).detach();

    return future;
}

template <typename ActionT>
std::shared_future<typename Client<ActionT>::WrappedResult>
Client<ActionT>::AsyncGetResult(
    std::shared_ptr<GoalHandle> goal_handle,
    typename GoalHandle::ResultCallback result_callback) {
    std::lock_guard<std::recursive_mutex> lock(goal_handles_mutex_);
    if (goal_handles_.count(goal_handle->GetGoalId()) == 0) {
        throw UnknownGoalHandleError();
    }

    if (result_callback) {
        goal_handle->SetResultCallback(result_callback);
    }

    // SetResultAwareness returns the *previous* value. If already true,
    // another GetResult poller is running — only return the shared future.
    if (goal_handle->SetResultAwareness(true)) {
        return goal_handle->AsyncGetResult();
    }

    std::thread([this, goal_handle]() {
        try {
            while (goal_handle->IsResultAware()) {
                auto request = std::make_shared<internal::GetResultRequest>();
                request->goal_id = goal_handle->GetGoalId();

                auto response_future =
                    get_result_client_->AsyncSendRequest(request);
                auto response = response_future.get();
                if (!response) {
                    AWARN << "AsyncGetResult: received null response for goal "
                             "ID: "
                          << ToString(goal_handle->GetGoalId());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }

                GoalStatus response_status =
                    static_cast<GoalStatus>(response->status);
                ADEBUG << "AsyncGetResult: received response for goal ID: "
                       << ToString(goal_handle->GetGoalId())
                       << " with status: " << static_cast<int>(response_status);

                if (response_status == GoalStatus::SUCCEEDED ||
                    response_status == GoalStatus::CANCELED ||
                    response_status == GoalStatus::ABORTED) {
                    WrappedResult wrapped_result;
                    wrapped_result.goal_id = goal_handle->GetGoalId();
                    wrapped_result.code =
                        static_cast<ResultCode>(response_status);
                    wrapped_result.result =
                        std::make_shared<Result>(response->result);

                    ADEBUG << "AsyncGetResult: setting result for goal ID: "
                           << ToString(goal_handle->GetGoalId())
                           << " with code: "
                           << static_cast<int>(wrapped_result.code);

                    goal_handle->SetResult(wrapped_result);

                    std::lock_guard<std::recursive_mutex> lock(
                        goal_handles_mutex_);
                    goal_handles_.erase(goal_handle->GetGoalId());
                    break;
                }

                ADEBUG << "AsyncGetResult: goal not in terminal state yet, "
                          "polling again";
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } catch (const std::exception& e) {
            AERROR << "Error in AsyncGetResult response: " << e.what();
            goal_handle->Invalidate(UnawareGoalHandleError());
        }
    }).detach();

    return goal_handle->AsyncGetResult();
}

template <typename ActionT>
std::shared_future<bool> Client<ActionT>::AsyncCancelGoal(
    std::shared_ptr<GoalHandle> goal_handle) {
    std::lock_guard<std::recursive_mutex> lock(goal_handles_mutex_);
    if (goal_handles_.count(goal_handle->GetGoalId()) == 0) {
        throw UnknownGoalHandleError();
    }

    auto promise = std::make_shared<std::promise<bool>>();
    std::shared_future<bool> future(promise->get_future());

    auto request = std::make_shared<internal::CancelGoalRequest>();
    request->goal_id = goal_handle->GetGoalId();

    auto response_future = cancel_goal_client_->AsyncSendRequest(request);
    std::thread([promise, response_future]() {
        try {
            auto response = response_future.get();
            promise->set_value(response && response->goals_canceling > 0);
        } catch (const std::exception& e) {
            AERROR << "Error in AsyncCancelGoal response: " << e.what();
            promise->set_value(false);
        }
    }).detach();

    return future;
}

template <typename ActionT>
bool Client<ActionT>::ActionServerIsReady() const {
    bool ready = send_goal_client_ && send_goal_client_->ServiceIsReady() &&
                 cancel_goal_client_ && cancel_goal_client_->ServiceIsReady() &&
                 get_result_client_ && get_result_client_->ServiceIsReady();
    ADEBUG << "ActionServerIsReady: send_goal="
           << (send_goal_client_ && send_goal_client_->ServiceIsReady())
           << ", cancel_goal="
           << (cancel_goal_client_ && cancel_goal_client_->ServiceIsReady())
           << ", get_result="
           << (get_result_client_ && get_result_client_->ServiceIsReady())
           << ", overall=" << ready;
    return ready;
}

template <typename ActionT>
void Client<ActionT>::HandleFeedback(
    const std::shared_ptr<internal::FeedbackMessage<ActionT>>& feedback_msg) {
    if (!feedback_msg) {
        AWARN << "HandleFeedback: received null feedback message";
        return;
    }

    std::lock_guard<std::recursive_mutex> guard(goal_handles_mutex_);
    const GoalUUID& goal_id = feedback_msg->goal_id;
    ADEBUG << "HandleFeedback: received feedback message for goal ID: "
           << ToString(goal_id);
    if (goal_handles_.count(goal_id) == 0) {
        AWARN << "HandleFeedback: received feedback for unknown goal ID: "
              << ToString(goal_id) << ". Ignoring...";
        return;
    }

    auto goal_handle = goal_handles_[goal_id].lock();
    if (!goal_handle) {
        ADEBUG << "HandleFeedback: goal handle expired for goal ID: "
               << ToString(goal_id) << ". Removing from map.";
        goal_handles_.erase(goal_id);
        return;
    }

    auto feedback = std::make_shared<Feedback>(feedback_msg->feedback);
    ADEBUG << "HandleFeedback: calling feedback callback for goal ID: "
           << ToString(goal_id) << ", progress: N/A";
    goal_handle->CallFeedbackCallback(goal_handle, feedback);
}

template <typename ActionT>
void Client<ActionT>::HandleStatus(
    const std::shared_ptr<internal::StatusMessage>& status_msg) {
    if (!status_msg) {
        return;
    }

    std::lock_guard<std::recursive_mutex> guard(goal_handles_mutex_);
    for (const auto& status_info : status_msg->status_list) {
        const GoalUUID& goal_id = status_info.goal_id;
        if (goal_handles_.count(goal_id) == 0) {
            continue;
        }

        auto goal_handle = goal_handles_[goal_id].lock();
        if (!goal_handle) {
            goal_handles_.erase(goal_id);
            continue;
        }

        goal_handle->SetStatus(status_info.status);
    }
}

}  // namespace action
}  // namespace autolink
