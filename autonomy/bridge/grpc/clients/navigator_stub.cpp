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

#include "autonomy/bridge/grpc/clients/navigator_stub.hpp"

#include "autolink/action/types.hpp"
#include "autonomy/bridge/constants.hpp"
#include "autonomy/bridge/grpc/clients/action_goal_session.hpp"
#include "autonomy/bridge/grpc/clients/stub_util.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace nav_proto = automsgs::actions;

proto::NavigationStatus ResolveNavigationStatus(
    autolink::action::ResultCode code) {
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

proto::TaskStatus ResolveNavigationTaskStatus(proto::NavigationStatus status) {
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

std::string ResolveBehaviorTree(
    const proto::NavigationCommandRequest& request) {
    if (request.has_plugins() && !request.plugins().behavior_tree().empty()) {
        return request.plugins().behavior_tree();
    }
    return {};
}

std::string DescribeAcceptFailure(ActionAcceptStatus status,
                                  const char* server_name) {
    switch (status) {
        case ActionAcceptStatus::kServerNotReady:
            return std::string(server_name) + " action server is not ready";
        case ActionAcceptStatus::kTimeout:
            return std::string("timeout waiting for ") + server_name +
                   " goal acceptance";
        case ActionAcceptStatus::kRejected:
            return std::string(server_name) + " goal rejected";
        default:
            return std::string(server_name) + " goal failed";
    }
}

}  // namespace

NavigatorStub::NavigatorStub(std::shared_ptr<autolink::Node> node,
                             std::shared_ptr<TaskMuxer> muxer)
    : navigate_to_pose_client_(std::make_shared<NavigateToPoseClient>(
          node, kNavigateToPoseActionServerName)),
      navigate_through_poses_client_(std::make_shared<NavigateThroughPosesClient>(
          node, kNavigateThroughPosesActionServerName)),
      muxer_(std::move(muxer)) {}

bool NavigatorStub::CheckServersReady() const {
    return navigate_to_pose_client_->ActionServerIsReady() &&
           navigate_through_poses_client_->ActionServerIsReady();
}

bool NavigatorStub::WaitForServers(const std::chrono::milliseconds timeout) {
    return navigate_to_pose_client_->WaitForServer(
               std::chrono::milliseconds(100), timeout) &&
           navigate_through_poses_client_->WaitForServer(
               std::chrono::milliseconds(100), timeout);
}

bool NavigatorStub::CheckNavigating() const {
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
    auto fail = [&](const std::string& message) {
        stream_callback(MakeResponse(request, proto::NAV_STATUS_FAILED, false,
                                     true, message));
    };
    if (RejectIfEstopActive(muxer_, fail)) {
        return false;
    }

    const auto command = request.command();
    if (command == proto::NAV_CMD_START) {
        if (RejectIfAcquireFailed(muxer_, proto::TASK_TYPE_NAVIGATION, request,
                                  fail)) {
            return false;
        }
        bool started = false;
        if (request.mode() == proto::NAV_MODE_SINGLE_POSE) {
            started = StartToPose(request, std::move(stream_callback));
        } else if (request.mode() == proto::NAV_MODE_THROUGH_POSES) {
            started = StartThroughPoses(request, std::move(stream_callback));
        } else {
            fail("missing or invalid navigation mode");
        }
        if (!started && muxer_) {
            muxer_->Release(proto::TASK_TYPE_NAVIGATION);
        }
        return started;
    }

    const bool handled = DispatchCommands(
        command,
        MakeCommandRules(
            [&] {
                return CancelActiveSession(request, std::move(stream_callback));
            },
            proto::NAV_CMD_CANCEL, proto::NAV_CMD_STOP),
        MakeCommandRule(proto::NAV_CMD_PAUSE, [&] {
            stream_callback(MakeResponse(request, proto::NAV_STATUS_PAUSED, true,
                                         false, "pause acknowledged"));
            return true;
        }),
        MakeCommandRule(proto::NAV_CMD_RESUME, [&] {
            stream_callback(MakeResponse(
                request, proto::NAV_STATUS_NAVIGATING, true, false,
                "resume acknowledged"));
            return true;
        }),
        MakeCommandRule(proto::NAV_CMD_REPLAN, [&] {
            stream_callback(MakeResponse(
                request, proto::NAV_STATUS_PLANNING, true, false,
                "replan acknowledged"));
            return true;
        }));

    if (handled) {
        return true;
    }
    fail("unsupported navigation command");
    return false;
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

    nav_proto::NavigateToPoseAction::Goal goal;
    *goal.mutable_pose() = request.goals(0);
    const std::string behavior_tree = ResolveBehaviorTree(request);
    if (!behavior_tree.empty()) {
        goal.set_behavior_tree(behavior_tree);
    }

    auto result = SendActionGoal(
        *navigate_to_pose_client_, goal,
        [this, request, stream_callback](
            const nav_proto::NavigateToPoseAction::Feedback& feedback) {
            auto response =
                MakeResponse(request, proto::NAV_STATUS_NAVIGATING, true, false);
            *response.mutable_current_pose() = feedback.current_pose();
            response.set_distance_remaining(feedback.distance_remaining());
            response.set_total_waypoints(1);
            response.set_current_waypoint_index(0);
            stream_callback(response);
        },
        [this, request, stream_callback](const auto& wrapped) {
            const auto status = ResolveNavigationStatus(wrapped.code);
            const bool success =
                wrapped.code == autolink::action::ResultCode::SUCCEEDED;
            std::string message;
            if (wrapped.result && !wrapped.result->error_msg().empty()) {
                message = wrapped.result->error_msg();
            }
            stream_callback(
                MakeResponse(request, status, success, true, message));
        },
        [this] {
            std::lock_guard<std::mutex> lock(mutex_);
            ClearActiveGoalLocked();
        });

    if (result.status != ActionAcceptStatus::kAccepted) {
        stream_callback(MakeResponse(
            request, proto::NAV_STATUS_FAILED, false, true,
            DescribeAcceptFailure(result.status, "navigate_to_pose")));
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_mode_ = ActiveMode::TO_POSE;
        to_pose_handle_ = result.handle;
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

    nav_proto::NavigateThroughPosesAction::Goal goal;
    goal.mutable_poses()->Reserve(request.goals_size());
    for (const auto& pose : request.goals()) {
        *goal.add_poses() = pose;
    }
    const std::string behavior_tree = ResolveBehaviorTree(request);
    if (!behavior_tree.empty()) {
        goal.set_behavior_tree(behavior_tree);
    }
    const int total_waypoints = request.goals_size();

    auto result = SendActionGoal(
        *navigate_through_poses_client_, goal,
        [this, request, stream_callback, total_waypoints](
            const nav_proto::NavigateThroughPosesAction::Feedback& feedback) {
            auto response =
                MakeResponse(request, proto::NAV_STATUS_NAVIGATING, true, false);
            *response.mutable_current_pose() = feedback.current_pose();
            response.set_distance_remaining(feedback.distance_remaining());
            response.set_total_waypoints(total_waypoints);
            response.set_current_waypoint_index(
                total_waypoints - feedback.number_of_poses_remaining());
            stream_callback(response);
        },
        [this, request, stream_callback](const auto& wrapped) {
            const auto status = ResolveNavigationStatus(wrapped.code);
            const bool success =
                wrapped.code == autolink::action::ResultCode::SUCCEEDED;
            std::string message;
            if (wrapped.result && !wrapped.result->error_msg().empty()) {
                message = wrapped.result->error_msg();
            }
            stream_callback(
                MakeResponse(request, status, success, true, message));
        },
        [this] {
            std::lock_guard<std::mutex> lock(mutex_);
            ClearActiveGoalLocked();
        });

    if (result.status != ActionAcceptStatus::kAccepted) {
        stream_callback(MakeResponse(
            request, proto::NAV_STATUS_FAILED, false, true,
            DescribeAcceptFailure(result.status, "navigate_through_poses")));
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_mode_ = ActiveMode::THROUGH_POSES;
        through_poses_handle_ = result.handle;
        total_waypoints_ = total_waypoints;
    }
    stream_callback(
        MakeResponse(request, proto::NAV_STATUS_NAVIGATING, true, false));
    return true;
}

bool NavigatorStub::CancelActiveSession(
    const proto::NavigationCommandRequest& request,
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
    stream_callback(
        MakeResponse(request, proto::NAV_STATUS_CANCELED, true, false));
    return true;
}

void NavigatorStub::ClearActiveGoalLocked() {
    active_mode_ = ActiveMode::NONE;
    to_pose_handle_.reset();
    through_poses_handle_.reset();
    total_waypoints_ = 0;
    if (muxer_) {
        muxer_->Release(proto::TASK_TYPE_NAVIGATION);
    }
}

proto::NavigationCommandResponse NavigatorStub::MakeResponse(
    const proto::NavigationCommandRequest& request,
    const proto::NavigationStatus status, const bool success, const bool final,
    const std::string& message) const {
    proto::NavigationCommandResponse response;
    response.set_status(status);
    FillCommandAck(response, proto::TASK_TYPE_NAVIGATION, request, success, final,
                   ResolveNavigationTaskStatus(status), message);
    if (status == proto::NAV_STATUS_NAVIGATING ||
        status == proto::NAV_STATUS_PLANNING) {
        response.set_total_waypoints(total_waypoints_);
    }
    return response;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
