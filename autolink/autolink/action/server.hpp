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
#include <string>
#include <unordered_map>
#include <utility>

#include "autolink/action/internal_types.hpp"
#include "autolink/action/server_goal_handle.hpp"
#include "autolink/action/types.hpp"
#include "autolink/common/log.hpp"
#include "autolink/common/macros.hpp"
#include "autolink/node/node.hpp"
#include "autolink/node/writer.hpp"
#include "autolink/service/service.hpp"
#include "autolink/time/time.hpp"

namespace autolink {
namespace action {

/**
 * @class Server
 * @brief Action server implementation.
 *
 * This class creates an action server that handles goal requests,
 * cancellations, and result requests. It uses autolink's Service and Channel
 * mechanisms.
 *
 * @tparam ActionT The action type, which should have Goal, Feedback, and
 * Result types defined.
 */
template <typename ActionT>
class Server
{
public:
    using Goal = typename ActionT::Goal;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;
    using GoalHandle = ServerGoalHandle<ActionT>;

    /// Signature of a callback that accepts or rejects goal requests.
    using GoalCallback = std::function<GoalResponse(
        const GoalUUID&, std::shared_ptr<const Goal>)>;

    /// Signature of a callback that accepts or rejects requests to cancel a
    /// goal.
    using CancelCallback =
        std::function<CancelResponse(std::shared_ptr<GoalHandle>)>;

    /// Signature of a callback that is used to notify when the goal has been
    /// accepted.
    using AcceptedCallback = std::function<void(std::shared_ptr<GoalHandle>)>;

    /**
     * @brief Defines shared_ptr and weak_ptr for Server.
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(Server)

    /**
     * @brief Construct a new Server object.
     *
     * @param node The node to create the server on.
     * @param action_name The name of the action.
     * @param handle_goal Callback to decide if a goal should be accepted.
     * @param handle_cancel Callback to decide if a goal should be canceled.
     * @param handle_accepted Callback called when a goal is accepted.
     */
    Server(std::shared_ptr<Node> node, const std::string& action_name,
           GoalCallback handle_goal, CancelCallback handle_cancel,
           AcceptedCallback handle_accepted)
        : node_(node),
          action_name_(action_name),
          handle_goal_(handle_goal),
          handle_cancel_(handle_cancel),
          handle_accepted_(handle_accepted) {
        Init();
    }

    virtual ~Server() {
        if (send_goal_service_) {
            send_goal_service_->destroy();
        }
        if (cancel_goal_service_) {
            cancel_goal_service_->destroy();
        }
        if (get_result_service_) {
            get_result_service_->destroy();
        }
    }

private:
    void Init();

    // Service callbacks
    void HandleSendGoal(
        const std::shared_ptr<internal::SendGoalRequest<ActionT>>& request,
        std::shared_ptr<internal::SendGoalResponse>& response);

    void HandleCancelGoal(
        const std::shared_ptr<internal::CancelGoalRequest>& request,
        std::shared_ptr<internal::CancelGoalResponse>& response);

    void HandleGetResult(
        const std::shared_ptr<internal::GetResultRequest>& request,
        std::shared_ptr<internal::GetResultResponse<ActionT>>& response);

    // Goal handle callbacks
    void OnTerminalState(const GoalUUID& uuid, std::shared_ptr<Result> result);
    void OnExecuting(const GoalUUID& uuid);
    void PublishFeedback(const GoalUUID& uuid,
                         std::shared_ptr<Feedback> feedback);

    // Helper methods
    void PublishStatus();
    std::shared_ptr<GoalHandle> GetGoalHandle(const GoalUUID& uuid);

    std::shared_ptr<Node> node_;
    std::string action_name_;

    GoalCallback handle_goal_;
    CancelCallback handle_cancel_;
    AcceptedCallback handle_accepted_;

    // Services
    std::shared_ptr<
        Service<internal::SendGoalRequest<ActionT>, internal::SendGoalResponse>>
        send_goal_service_;
    std::shared_ptr<
        Service<internal::CancelGoalRequest, internal::CancelGoalResponse>>
        cancel_goal_service_;
    std::shared_ptr<Service<internal::GetResultRequest,
                            internal::GetResultResponse<ActionT>>>
        get_result_service_;

    // Publishers
    std::shared_ptr<Writer<internal::FeedbackMessage<ActionT>>>
        feedback_writer_;
    std::shared_ptr<Writer<internal::StatusMessage>> status_writer_;

    // Goal handles
    std::unordered_map<GoalUUID, std::weak_ptr<GoalHandle>> goal_handles_;
    std::mutex goal_handles_mutex_;

    // Results storage (stores both result and final status)
    struct StoredResult {
        std::shared_ptr<Result> result;
        GoalStatus status;
    };
    std::unordered_map<GoalUUID, StoredResult> results_;
    std::mutex results_mutex_;
};

template <typename ActionT>
void Server<ActionT>::Init() {
    std::string node_name = node_->Name();
    AINFO << "Server::Init: initializing action server for: " << action_name_;

    // Create SendGoal service
    send_goal_service_ = node_->CreateService<
        internal::SendGoalRequest<ActionT>, internal::SendGoalResponse>(
        action_name_ + "/send_goal",
        [this](const std::shared_ptr<internal::SendGoalRequest<ActionT>>& req,
               std::shared_ptr<internal::SendGoalResponse>& res) {
            this->HandleSendGoal(req, res);
        });

    if (!send_goal_service_) {
        AERROR << "Server::Init: failed to create send_goal service";
        return;
    }
    AINFO << "Server::Init: send_goal service created";

    // Create CancelGoal service
    cancel_goal_service_ = node_->CreateService<internal::CancelGoalRequest,
                                                internal::CancelGoalResponse>(
        action_name_ + "/cancel_goal",
        [this](const std::shared_ptr<internal::CancelGoalRequest>& req,
               std::shared_ptr<internal::CancelGoalResponse>& res) {
            this->HandleCancelGoal(req, res);
        });

    if (!cancel_goal_service_) {
        AERROR << "Server::Init: failed to create cancel_goal service";
        return;
    }
    AINFO << "Server::Init: cancel_goal service created";

    // Create GetResult service
    get_result_service_ =
        node_->CreateService<internal::GetResultRequest,
                             internal::GetResultResponse<ActionT>>(
            action_name_ + "/get_result",
            [this](const std::shared_ptr<internal::GetResultRequest>& req,
                   std::shared_ptr<internal::GetResultResponse<ActionT>>& res) {
                this->HandleGetResult(req, res);
            });

    if (!get_result_service_) {
        AERROR << "Server::Init: failed to create get_result service";
        return;
    }
    AINFO << "Server::Init: get_result service created";

    // Create feedback writer
    std::string feedback_channel = action_name_ + "/feedback";
    AINFO << "Server::Init: Creating feedback writer for channel: "
          << feedback_channel;
    feedback_writer_ = node_->CreateWriter<internal::FeedbackMessage<ActionT>>(
        feedback_channel);
    if (feedback_writer_) {
        AINFO << "Server::Init: Feedback writer created successfully";
    } else {
        AERROR << "Server::Init: Failed to create feedback writer";
    }

    // Create status writer
    status_writer_ =
        node_->CreateWriter<internal::StatusMessage>(action_name_ + "/status");
    AINFO << "Server::Init: status writer created";
    AINFO << "Server::Init: all services and writers initialized successfully";
}

template <typename ActionT>
void Server<ActionT>::HandleSendGoal(
    const std::shared_ptr<internal::SendGoalRequest<ActionT>>& request,
    std::shared_ptr<internal::SendGoalResponse>& response) {
    AINFO << "Server::HandleSendGoal: ENTERED - checking request and response";
    if (!request || !response) {
        AERROR << "Server::HandleSendGoal: Invalid request or response";
        return;
    }

    GoalUUID goal_id = request->goal_id;
    AINFO << "Server::HandleSendGoal: received goal request for goal ID: "
          << ToString(goal_id);

    auto goal = std::make_shared<const Goal>(request->goal);

    // Call user's goal callback
    GoalResponse goal_response = handle_goal_(goal_id, goal);
    ADEBUG << "HandleSendGoal: goal callback returned: "
           << static_cast<int>(goal_response);

    response->accepted = (goal_response == GoalResponse::ACCEPT_AND_EXECUTE ||
                          goal_response == GoalResponse::ACCEPT_AND_DEFER);
    response->stamp = Time::Now().ToNanosecond();

    ADEBUG << "HandleSendGoal: response accepted=" << response->accepted
           << " for goal ID: " << ToString(goal_id);

    if (response->accepted) {
        // Create goal handle
        // Use std::bind to avoid lambda type issues in C++11
        auto on_terminal =
            std::bind(&Server<ActionT>::OnTerminalState, this,
                      std::placeholders::_1, std::placeholders::_2);
        auto on_exec = std::bind(&Server<ActionT>::OnExecuting, this,
                                 std::placeholders::_1);
        auto pub_feedback = std::bind(&Server<ActionT>::PublishFeedback, this,
                                      goal_id, std::placeholders::_1);

        auto goal_handle = std::make_shared<GoalHandle>(
            goal_id, goal, on_terminal, on_exec, pub_feedback);

        {
            std::lock_guard<std::mutex> lock(goal_handles_mutex_);
            goal_handles_[goal_id] = goal_handle;
        }

        // Call user's accepted callback
        if (handle_accepted_) {
            handle_accepted_(goal_handle);
        }

        // Publish status update
        PublishStatus();
    }
}

template <typename ActionT>
void Server<ActionT>::HandleCancelGoal(
    const std::shared_ptr<internal::CancelGoalRequest>& request,
    std::shared_ptr<internal::CancelGoalResponse>& response) {
    if (!request || !response) {
        AERROR << "Invalid request or response in HandleCancelGoal";
        return;
    }

    GoalUUID goal_id = request->goal_id;
    auto goal_handle = GetGoalHandle(goal_id);

    int32_t goals_canceling = 0;
    if (goal_handle) {
        CancelResponse cancel_response = handle_cancel_(goal_handle);
        if (cancel_response == CancelResponse::ACCEPT) {
            if (goal_handle->NotifyCancelRequestAccepted()) {
                goals_canceling = 1;
            } else if (goal_handle->IsCanceling()) {
                goals_canceling = 1;
            }
        }
    }

    response->goals_canceling = goals_canceling;
    PublishStatus();
}

template <typename ActionT>
void Server<ActionT>::HandleGetResult(
    const std::shared_ptr<internal::GetResultRequest>& request,
    std::shared_ptr<internal::GetResultResponse<ActionT>>& response) {
    if (!request || !response) {
        AERROR << "Invalid request or response in HandleGetResult";
        return;
    }

    GoalUUID goal_id = request->goal_id;
    ADEBUG << "HandleGetResult: request for goal ID: " << ToString(goal_id);

    // First check if result is already stored (goal completed)
    {
        std::lock_guard<std::mutex> lock(results_mutex_);
        if (results_.count(goal_id) > 0) {
            const StoredResult& stored = results_[goal_id];
            response->result = *stored.result;
            response->status = static_cast<int8_t>(stored.status);
            ADEBUG << "HandleGetResult: found result in storage for goal ID: "
                   << ToString(goal_id)
                   << " with status: " << static_cast<int>(stored.status);
            return;
        }
    }

    // If result not found, check goal_handle status
    std::shared_ptr<GoalHandle> goal_handle = GetGoalHandle(goal_id);
    if (goal_handle) {
        GoalStatus status = goal_handle->GetStatus();
        response->status = static_cast<int8_t>(status);

        // If goal is in terminal state but result not in storage yet,
        // return default result (this shouldn't happen normally)
        if (status == GoalStatus::SUCCEEDED || status == GoalStatus::CANCELED ||
            status == GoalStatus::ABORTED) {
            ADEBUG << "HandleGetResult: goal in terminal state but result not "
                      "in storage. "
                   << "Status: " << static_cast<int>(status);
            // Result should be in storage, but if not, return default
            // The result will be available on next request
        } else {
            ADEBUG << "HandleGetResult: goal not in terminal state yet. "
                   << "Status: " << static_cast<int>(status);
        }
    } else {
        // Goal handle not found
        AWARN << "HandleGetResult: goal handle not found for goal ID: "
              << ToString(goal_id);
        response->status = static_cast<int8_t>(GoalStatus::UNKNOWN);
    }
}

template <typename ActionT>
void Server<ActionT>::OnTerminalState(const GoalUUID& uuid,
                                      std::shared_ptr<Result> result) {
    // Get status from goal_handle before erasing it
    GoalStatus final_status = GoalStatus::UNKNOWN;
    {
        std::lock_guard<std::mutex> lock(goal_handles_mutex_);
        auto it = goal_handles_.find(uuid);
        if (it != goal_handles_.end()) {
            auto goal_handle = it->second.lock();
            if (goal_handle) {
                final_status = goal_handle->GetStatus();
            }
            goal_handles_.erase(uuid);
        }
    }

    // Store result with status
    {
        std::lock_guard<std::mutex> lock(results_mutex_);
        StoredResult stored;
        stored.result = result;
        stored.status = final_status;
        results_[uuid] = stored;
        ADEBUG << "OnTerminalState: stored result for goal ID: "
               << ToString(uuid)
               << " with status: " << static_cast<int>(final_status);
    }

    PublishStatus();
}

template <typename ActionT>
void Server<ActionT>::OnExecuting(const GoalUUID& uuid) {
    AINFO << "Server::OnExecuting: ENTERED for goal ID: " << ToString(uuid);
    PublishStatus();
    AINFO << "Server::OnExecuting: PublishStatus() returned, EXITING";
}

template <typename ActionT>
void Server<ActionT>::PublishFeedback(const GoalUUID& uuid,
                                      std::shared_ptr<Feedback> feedback) {
    if (!feedback_writer_ || !feedback) {
        AWARN
            << "PublishFeedback: feedback_writer_ is null or feedback is null";
        return;
    }

    // Always try to publish feedback, even if HasReader() returns false
    // The reader might not be discovered yet, but we should still try
    bool has_reader = feedback_writer_->HasReader();
    ADEBUG << "PublishFeedback: attempting to publish feedback for goal ID: "
           << ToString(uuid) << ", HasReader()=" << has_reader;

    auto feedback_msg = std::make_shared<internal::FeedbackMessage<ActionT>>();
    feedback_msg->goal_id = uuid;
    feedback_msg->feedback = *feedback;

    // Try to write even if HasReader() is false - reader might not be
    // discovered yet
    bool success = feedback_writer_->Write(feedback_msg);
    if (!success) {
        AWARN
            << "PublishFeedback: failed to write feedback message for goal ID: "
            << ToString(uuid) << ", HasReader()=" << has_reader;
    } else {
        ADEBUG
            << "PublishFeedback: successfully published feedback for goal ID: "
            << ToString(uuid) << ", progress: N/A";
    }
}

template <typename ActionT>
void Server<ActionT>::PublishStatus() {
    if (!status_writer_) {
        return;
    }

    auto status_msg = std::make_shared<internal::StatusMessage>();
    {
        std::lock_guard<std::mutex> lock(goal_handles_mutex_);
        for (const auto& [uuid, weak_handle] : goal_handles_) {
            auto handle = weak_handle.lock();
            if (handle) {
                GoalStatus status = handle->GetStatus();
                internal::GoalStatusInfo info;
                info.goal_id = uuid;
                info.status = static_cast<int8_t>(status);
                status_msg->status_list.push_back(info);
            }
        }
    }

    status_writer_->Write(status_msg);
}

template <typename ActionT>
std::shared_ptr<typename Server<ActionT>::GoalHandle>
Server<ActionT>::GetGoalHandle(const GoalUUID& uuid) {
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);
    auto it = goal_handles_.find(uuid);
    if (it != goal_handles_.end()) {
        return it->second.lock();
    }
    return nullptr;
}

}  // namespace action
}  // namespace autolink
