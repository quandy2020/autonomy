/*
 * Copyright 2026 The Openbot Authors
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

#include "autonomy/task/common/navigator.hpp"

#include <thread>

#include "autolink/autolink.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/task/common/names.hpp"
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>

namespace autonomy {
namespace task {
namespace common {
namespace {

namespace task_proto = ::autonomy::task::proto;
namespace status_msgs = ::automsgs::msgs::status_msgs;

void SetElapsed(std::chrono::steady_clock::time_point start_time,
                ::automsgs::msgs::builtin_interfaces::Duration* out) {
    if (out == nullptr) {
        return;
    }
    const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - start_time);
    const auto seconds =
        std::chrono::duration_cast<std::chrono::seconds>(nanoseconds);
    out->set_sec(static_cast<int32_t>(seconds.count()));
    out->set_nanosec(static_cast<int32_t>((nanoseconds - seconds).count()));
}

}  // namespace

Navigator::~Navigator() { Stop(); }

bool Navigator::Start(const std::shared_ptr<autolink::Node>& node,
                      SubmitGoalFunction submit_goal,
                      FeedbackFunction feedback, ResultFunction result,
                      std::chrono::milliseconds period) {
    Stop();
    if (!node || !submit_goal || !feedback || !result) {
        return false;
    }
    node_ = node;
    submit_goal_ = std::move(submit_goal);
    feedback_ = std::move(feedback);
    result_ = std::move(result);
    period_ = period.count() > 0 ? period : std::chrono::milliseconds(100);

    navigate_to_pose_server_ = std::make_shared<NavigateToPoseServer>(
        node_, kNavigateToPose, [this] { RunNavigateToPose(); });
    navigate_through_poses_server_ =
        std::make_shared<NavigateThroughPosesServer>(
            node_, kNavigateThroughPoses,
            [this] { RunNavigateThroughPoses(); });

    AINFO << "Navigator started: " << kNavigateToPose << ", "
          << kNavigateThroughPoses;
    return true;
}

void Navigator::Stop() {
    navigate_to_pose_server_.reset();
    navigate_through_poses_server_.reset();
    node_.reset();
    submit_goal_ = nullptr;
    feedback_ = nullptr;
    result_ = nullptr;
}

bool Navigator::AcceptNavigateToPose(
    const NavigateToPoseAction::Goal& action_goal) {
    task_proto::NavigationGoal goal;
    goal.set_command(task_proto::NAV_CMD_START);
    goal.set_mode(task_proto::NAV_MODE_SINGLE_POSE);
    *goal.add_goals() = action_goal.pose();
    if (!action_goal.behavior_tree().empty()) {
        goal.mutable_plugins()->set_behavior_tree(action_goal.behavior_tree());
    }
    return submit_goal_(goal);
}

bool Navigator::AcceptNavigateThroughPoses(
    const NavigateThroughPosesAction::Goal& action_goal) {
    task_proto::NavigationGoal goal;
    goal.set_command(task_proto::NAV_CMD_START);
    goal.set_mode(task_proto::NAV_MODE_THROUGH_POSES);
    goal.mutable_goals()->Reserve(action_goal.poses_size());
    for (const auto& pose : action_goal.poses()) {
        *goal.add_goals() = pose;
    }
    if (!action_goal.behavior_tree().empty()) {
        goal.mutable_plugins()->set_behavior_tree(action_goal.behavior_tree());
    }
    return submit_goal_(goal);
}

bool Navigator::Cancel() {
    task_proto::NavigationGoal goal;
    goal.set_command(task_proto::NAV_CMD_CANCEL);
    goal.set_mode(task_proto::NAV_MODE_UNSPECIFIED);
    return submit_goal_(goal);
}

status_msgs::StatusCode Navigator::ToStatusCode(
    const task_proto::NavigationStatus status, const bool rejected) {
    if (rejected) {
        return status_msgs::NAVIGATION_GOAL_REJECTED;
    }
    switch (status) {
        case task_proto::NAV_STATUS_SUCCEEDED:
            return status_msgs::OK;
        case task_proto::NAV_STATUS_CANCELED:
            return status_msgs::NAVIGATION_CANCELLED;
        case task_proto::NAV_STATUS_FAILED:
            return status_msgs::TASK_FAILED;
        default:
            return status_msgs::UNKNOWN;
    }
}

void Navigator::RunNavigateToPose() {
    Spin<NavigateToPoseServer, NavigateToPoseAction::Feedback,
         NavigateToPoseAction::Result>(
        navigate_to_pose_server_,
        [this] {
            const auto goal = navigate_to_pose_server_->GetCurrentGoal();
            return goal && AcceptNavigateToPose(*goal);
        },
        [](const task_proto::NavigationFeedback& in,
           NavigateToPoseAction::Feedback* out) {
            if (out == nullptr) {
                return;
            }
            if (in.has_current_pose()) {
                *out->mutable_current_pose() = in.current_pose();
            }
            out->set_distance_remaining(in.distance_remaining());
        });
}

void Navigator::RunNavigateThroughPoses() {
    Spin<NavigateThroughPosesServer, NavigateThroughPosesAction::Feedback,
         NavigateThroughPosesAction::Result>(
        navigate_through_poses_server_,
        [this] {
            const auto goal =
                navigate_through_poses_server_->GetCurrentGoal();
            return goal && AcceptNavigateThroughPoses(*goal);
        },
        [](const task_proto::NavigationFeedback& in,
           NavigateThroughPosesAction::Feedback* out) {
            if (out == nullptr) {
                return;
            }
            if (in.has_current_pose()) {
                *out->mutable_current_pose() = in.current_pose();
            }
            out->set_distance_remaining(in.distance_remaining());
            const int total = in.total_waypoints();
            const int index = in.current_waypoint_index();
            out->set_number_of_poses_remaining(total > index ? total - index
                                                             : 0);
        });
}

template <typename ServerType, typename FeedbackType, typename ResultType>
void Navigator::Spin(
    const std::shared_ptr<ServerType>& server,
    std::function<bool()> accept_goal,
    std::function<void(const task_proto::NavigationFeedback&, FeedbackType*)>
        fill_feedback) {
    if (!server || !submit_goal_ || !feedback_ || !result_) {
        return;
    }
    if (!accept_goal()) {
        auto rejected = std::make_shared<ResultType>();
        rejected->set_error_code(
            ToStatusCode(task_proto::NAV_STATUS_FAILED, true));
        rejected->set_error_msg("navigation goal rejected");
        server->TerminateCurrent(rejected);
        return;
    }

    const auto start_time = std::chrono::steady_clock::now();
    while (autolink::OK()) {
        if (server->IsCancelRequested()) {
            Cancel();
        }
        if (server->IsPreemptRequested()) {
            Cancel();
            server->AcceptPendingGoal();
            return;
        }

        task_proto::NavigationResult task_result;
        if (result_(&task_result)) {
            auto out = std::make_shared<ResultType>();
            out->set_error_code(
                ToStatusCode(task_result.final_status(), false));
            out->set_error_msg(task_result.result().message());
            if (task_result.final_status() ==
                task_proto::NAV_STATUS_SUCCEEDED) {
                server->SucceededCurrent(out);
            } else {
                server->TerminateCurrent(out);
            }
            return;
        }

        task_proto::NavigationFeedback task_feedback;
        if (feedback_(&task_feedback)) {
            auto out = std::make_shared<FeedbackType>();
            fill_feedback(task_feedback, out.get());
            SetElapsed(start_time, out->mutable_navigation_time());
            server->PublishFeedback(out);
        }
        std::this_thread::sleep_for(period_);
    }

    auto aborted = std::make_shared<ResultType>();
    aborted->set_error_code(status_msgs::UNKNOWN);
    aborted->set_error_msg("Navigator stopped");
    server->TerminateCurrent(aborted);
}

template void Navigator::Spin<
    Navigator::NavigateToPoseServer,
    ::automsgs::actions::NavigateToPoseAction::Feedback,
    ::automsgs::actions::NavigateToPoseAction::Result>(
    const std::shared_ptr<NavigateToPoseServer>&, std::function<bool()>,
    std::function<void(
        const task_proto::NavigationFeedback&,
        ::automsgs::actions::NavigateToPoseAction::Feedback*)>);

template void Navigator::Spin<
    Navigator::NavigateThroughPosesServer,
    ::automsgs::actions::NavigateThroughPosesAction::Feedback,
    ::automsgs::actions::NavigateThroughPosesAction::Result>(
    const std::shared_ptr<NavigateThroughPosesServer>&, std::function<bool()>,
    std::function<void(
        const task_proto::NavigationFeedback&,
        ::automsgs::actions::NavigateThroughPosesAction::Feedback*)>);

}  // namespace common
}  // namespace task
}  // namespace autonomy
