/*
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

#include "autonomy/bridge/plugins/grpc/clients/navigator_stub.hpp"

#include "autolink/action/types.hpp"
#include "autonomy/bridge/constants.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace bridge {
namespace plugins {
namespace grpc {
namespace clients {
namespace {

namespace nav_proto = automsgs::actions;

proto::NavigationStatus ToNavigationStatus(autolink::action::ResultCode code) {
    switch (code) {
        case autolink::action::ResultCode::SUCCEEDED:
            return proto::NAV_STATUS_SUCCEEDED;
        case autolink::action::ResultCode::CANCELED:
            return proto::NAV_STATUS_CANCELED;
        case autolink::action::ResultCode::ABORTED:
            return proto::NAV_STATUS_FAILED;
        default:
            return proto::NAV_STATUS_UNKNOWN;
    }
}

proto::TaskStatus ToTaskStatus(proto::NavigationStatus status) {
    switch (status) {
        case proto::NAV_STATUS_NAVIGATING:
        case proto::NAV_STATUS_PLANNING:
            return proto::TASK_STATUS_RUNNING;
        case proto::NAV_STATUS_SUCCEEDED:
            return proto::TASK_STATUS_SUCCEEDED;
        case proto::NAV_STATUS_FAILED:
            return proto::TASK_STATUS_FAILED;
        case proto::NAV_STATUS_CANCELED:
            return proto::TASK_STATUS_CANCELED;
        case proto::NAV_STATUS_PAUSED:
            return proto::TASK_STATUS_PAUSED;
        default:
            return proto::TASK_STATUS_IDLE;
    }
}

std::string BehaviorTreeFromRequest(
    const proto::NavigationCommandRequest& request) {
    if (request.has_plugins() && !request.plugins().behavior_tree().empty()) {
        return request.plugins().behavior_tree();
    }
    return {};
}

}  // namespace

NavigatorStub::NavigatorStub(std::shared_ptr<autolink::Node> node)
    : navigate_to_pose_client_(std::make_shared<NavigateToPoseClient>(
          node, kNavigateToPoseActionServerName)),
      navigate_through_poses_client_(std::make_shared<NavigateThroughPosesClient>(
          node, kNavigateThroughPosesActionServerName)) {}

bool NavigatorStub::ServersReady() const {
    return navigate_to_pose_client_->ActionServerIsReady() &&
           navigate_through_poses_client_->ActionServerIsReady();
}

bool NavigatorStub::WaitForServers(const std::chrono::milliseconds timeout) {
    return navigate_to_pose_client_->WaitForServer(
               std::chrono::milliseconds(100), timeout) &&
           navigate_through_poses_client_->WaitForServer(
               std::chrono::milliseconds(100), timeout);
}

bool NavigatorStub::IsNavigating() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_mode_ != ActiveMode::NONE;
}

bool NavigatorStub::HandleCommand(
    const proto::NavigationCommandRequest& request,
    StreamCallback stream_callback) {
    if (!stream_callback) {
        AERROR << "NavigatorStub: stream callback is null.";
        return false;
    }

    switch (request.command()) {
        case proto::NAV_CMD_START:
            if (request.mode() == proto::NAV_MODE_SINGLE_POSE) {
                return StartToPose(request, std::move(stream_callback));
            }
            if (request.mode() == proto::NAV_MODE_THROUGH_POSES) {
                return StartThroughPoses(request, std::move(stream_callback));
            }
            stream_callback(MakeResponse(request, proto::NAV_STATUS_FAILED, false,
                                         true, "missing or invalid navigation mode"));
            return false;
        case proto::NAV_CMD_CANCEL:
        case proto::NAV_CMD_STOP:
            return CancelActive(request, std::move(stream_callback));
        case proto::NAV_CMD_PAUSE:
        case proto::NAV_CMD_RESUME:
        case proto::NAV_CMD_REPLAN:
            stream_callback(MakeResponse(
                request, proto::NAV_STATUS_FAILED, false, true,
                "navigation command not implemented yet"));
            return false;
        default:
            stream_callback(MakeResponse(request, proto::NAV_STATUS_FAILED, false,
                                         true, "unknown navigation command"));
            return false;
    }
}

bool NavigatorStub::StartToPose(const proto::NavigationCommandRequest& request,
                                StreamCallback stream_callback) {
    if (request.goals_size() < 1) {
        stream_callback(MakeResponse(request, proto::NAV_STATUS_FAILED, false, true,
                                     "navigate_to_pose requires at least one goal"));
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_mode_ != ActiveMode::NONE) {
            stream_callback(MakeResponse(
                request, proto::NAV_STATUS_FAILED, false, true,
                "another navigation goal is already active"));
            return false;
        }
    }

    if (!navigate_to_pose_client_->ActionServerIsReady()) {
        stream_callback(MakeResponse(
            request, proto::NAV_STATUS_FAILED, false, true,
            "navigate_to_pose action server is not ready"));
        return false;
    }

    nav_proto::NavigateToPoseAction::Goal goal;
    *goal.mutable_pose() = request.goals(0);
    const std::string behavior_tree = BehaviorTreeFromRequest(request);
    if (!behavior_tree.empty()) {
        goal.set_behavior_tree(behavior_tree);
    }

    NavigateToPoseClient::SendGoalOptions options;
    options.feedback_callback =
        [this, request, stream_callback = stream_callback](
            std::shared_ptr<ToPoseGoalHandle>,
            std::shared_ptr<const nav_proto::NavigateToPoseAction::Feedback>
                feedback) {
            if (!feedback) {
                return;
            }
            auto response =
                MakeResponse(request, proto::NAV_STATUS_NAVIGATING, true, false);
            *response.mutable_current_pose() = feedback->current_pose();
            response.set_distance_remaining(feedback->distance_remaining());
            response.set_total_waypoints(1);
            response.set_current_waypoint_index(0);
            stream_callback(response);
        };
    options.result_callback =
        [this, request, stream_callback = stream_callback](
            const ToPoseGoalHandle::WrappedResult& wrapped) {
            const auto status = ToNavigationStatus(wrapped.code);
            const bool success =
                wrapped.code == autolink::action::ResultCode::SUCCEEDED;
            std::string message;
            if (wrapped.result && !wrapped.result->error_msg().empty()) {
                message = wrapped.result->error_msg();
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                ClearActiveGoalLocked();
            }
            stream_callback(
                MakeResponse(request, status, success, true, message));
        };

    const auto accepted_future = navigate_to_pose_client_->AsyncSendGoal(goal, options);
    if (accepted_future.wait_for(std::chrono::seconds(30)) !=
        std::future_status::ready) {
        stream_callback(MakeResponse(
            request, proto::NAV_STATUS_FAILED, false, true,
            "timeout waiting for navigate_to_pose goal acceptance"));
        return false;
    }

    const auto goal_handle = accepted_future.get();
    if (!goal_handle) {
        stream_callback(MakeResponse(
            request, proto::NAV_STATUS_FAILED, false, true,
            "navigate_to_pose goal rejected"));
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_mode_ = ActiveMode::TO_POSE;
        to_pose_handle_ = goal_handle;
        total_waypoints_ = 1;
    }

    stream_callback(
        MakeResponse(request, proto::NAV_STATUS_NAVIGATING, true, false));
    return true;
}

bool NavigatorStub::StartThroughPoses(
    const proto::NavigationCommandRequest& request,
    StreamCallback stream_callback) {
    if (request.goals_size() < 1) {
        stream_callback(MakeResponse(
            request, proto::NAV_STATUS_FAILED, false, true,
            "navigate_through_poses requires at least one goal"));
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_mode_ != ActiveMode::NONE) {
            stream_callback(MakeResponse(
                request, proto::NAV_STATUS_FAILED, false, true,
                "another navigation goal is already active"));
            return false;
        }
    }

    if (!navigate_through_poses_client_->ActionServerIsReady()) {
        stream_callback(MakeResponse(
            request, proto::NAV_STATUS_FAILED, false, true,
            "navigate_through_poses action server is not ready"));
        return false;
    }

    nav_proto::NavigateThroughPosesAction::Goal goal;
    goal.mutable_poses()->Reserve(request.goals_size());
    for (const auto& pose : request.goals()) {
        *goal.add_poses() = pose;
    }
    const std::string behavior_tree = BehaviorTreeFromRequest(request);
    if (!behavior_tree.empty()) {
        goal.set_behavior_tree(behavior_tree);
    }

    const int total_waypoints = request.goals_size();

    NavigateThroughPosesClient::SendGoalOptions options;
    options.feedback_callback =
        [this, request, stream_callback, total_waypoints](
            std::shared_ptr<ThroughPosesGoalHandle>,
            std::shared_ptr<const nav_proto::NavigateThroughPosesAction::Feedback>
                feedback) {
            if (!feedback) {
                return;
            }
            auto response =
                MakeResponse(request, proto::NAV_STATUS_NAVIGATING, true, false);
            *response.mutable_current_pose() = feedback->current_pose();
            response.set_distance_remaining(feedback->distance_remaining());
            response.set_total_waypoints(total_waypoints);
            response.set_current_waypoint_index(
                total_waypoints - feedback->number_of_poses_remaining());
            stream_callback(response);
        };
    options.result_callback =
        [this, request, stream_callback](
            const ThroughPosesGoalHandle::WrappedResult& wrapped) {
            const auto status = ToNavigationStatus(wrapped.code);
            const bool success =
                wrapped.code == autolink::action::ResultCode::SUCCEEDED;
            std::string message;
            if (wrapped.result && !wrapped.result->error_msg().empty()) {
                message = wrapped.result->error_msg();
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                ClearActiveGoalLocked();
            }
            stream_callback(
                MakeResponse(request, status, success, true, message));
        };

    const auto accepted_future =
        navigate_through_poses_client_->AsyncSendGoal(goal, options);
    if (accepted_future.wait_for(std::chrono::seconds(30)) !=
        std::future_status::ready) {
        stream_callback(MakeResponse(
            request, proto::NAV_STATUS_FAILED, false, true,
            "timeout waiting for navigate_through_poses goal acceptance"));
        return false;
    }

    const auto goal_handle = accepted_future.get();
    if (!goal_handle) {
        stream_callback(MakeResponse(
            request, proto::NAV_STATUS_FAILED, false, true,
            "navigate_through_poses goal rejected"));
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_mode_ = ActiveMode::THROUGH_POSES;
        through_poses_handle_ = goal_handle;
        total_waypoints_ = total_waypoints;
    }

    stream_callback(
        MakeResponse(request, proto::NAV_STATUS_NAVIGATING, true, false));
    return true;
}

bool NavigatorStub::CancelActive(const proto::NavigationCommandRequest& request,
                                 StreamCallback stream_callback) {
    ActiveMode mode = ActiveMode::NONE;
    std::shared_ptr<ToPoseGoalHandle> to_pose_handle;
    std::shared_ptr<ThroughPosesGoalHandle> through_poses_handle;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        mode = active_mode_;
        to_pose_handle = to_pose_handle_;
        through_poses_handle = through_poses_handle_;
    }

    if (mode == ActiveMode::NONE) {
        stream_callback(MakeResponse(request, proto::NAV_STATUS_IDLE, true, true,
                                     "no active navigation goal"));
        return true;
    }

    std::shared_future<bool> cancel_future;
    if (mode == ActiveMode::TO_POSE && to_pose_handle) {
        cancel_future = navigate_to_pose_client_->AsyncCancelGoal(to_pose_handle);
    } else if (mode == ActiveMode::THROUGH_POSES && through_poses_handle) {
        cancel_future =
            navigate_through_poses_client_->AsyncCancelGoal(through_poses_handle);
    } else {
        stream_callback(MakeResponse(request, proto::NAV_STATUS_FAILED, false, true,
                                     "active goal handle missing"));
        return false;
    }

    if (cancel_future.wait_for(std::chrono::seconds(10)) !=
        std::future_status::ready) {
        stream_callback(MakeResponse(request, proto::NAV_STATUS_FAILED, false, true,
                                     "timeout canceling navigation goal"));
        return false;
    }

    if (!cancel_future.get()) {
        stream_callback(MakeResponse(request, proto::NAV_STATUS_FAILED, false, true,
                                     "cancel request rejected"));
        return false;
    }

    // Terminal NAV_STATUS_CANCELED arrives via result_callback when the server
    // finishes canceling; emit an interim ack for the RPC stream.
    stream_callback(
        MakeResponse(request, proto::NAV_STATUS_CANCELED, true, false));
    return true;
}

void NavigatorStub::ClearActiveGoalLocked() {
    active_mode_ = ActiveMode::NONE;
    to_pose_handle_.reset();
    through_poses_handle_.reset();
    total_waypoints_ = 0;
}

proto::NavigationCommandResponse NavigatorStub::MakeResponse(
    const proto::NavigationCommandRequest& request,
    const proto::NavigationStatus status, const bool success, const bool final,
    const std::string& message) const {
    proto::NavigationCommandResponse response;
    response.set_status(status);

    auto* ack = response.mutable_ack();
    ack->set_success(success);
    ack->set_final(final);
    if (request.has_header()) {
        ack->set_cmd_id(request.header().cmd_id());
    }
    ack->set_task_type(proto::TASK_TYPE_NAVIGATION);
    ack->set_task_status(ToTaskStatus(status));
    if (!message.empty()) {
        ack->set_message(message);
    }

    if (status == proto::NAV_STATUS_NAVIGATING ||
        status == proto::NAV_STATUS_PLANNING) {
        response.set_total_waypoints(total_waypoints_);
    }

    return response;
}

}  // namespace clients
}  // namespace grpc
}  // namespace plugins
}  // namespace bridge
}  // namespace autonomy
