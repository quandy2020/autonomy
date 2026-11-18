/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file navigator.cpp
 * @brief Tick-based staging navigation (implementation).
 */

#include "autonomy/task/charging/dock/navigator.hpp"

#include <automsgs/actions/nav_actions.pb.h>
#include "autonomy/common/logging.hpp"
#include "autonomy/task/charging/dock/exceptions.hpp"

namespace autonomy {
namespace task {
namespace charging {

void DockNavigator::Configure(navigation::NavigationClient::Ptr client,
                              std::string planner_id, std::string controller_id,
                              std::string smoother_id) {
    nav_ = std::move(client);
    if (!planner_id.empty()) {
        planner_id_ = std::move(planner_id);
    }
    if (!controller_id.empty()) {
        controller_id_ = std::move(controller_id);
    }
    if (!smoother_id.empty()) {
        smoother_id_ = std::move(smoother_id);
    }
}

bool DockNavigator::LaunchFollow() {
    automsgs::msgs::nav_msgs::Path path;
    if (!nav_->ComputePathToPose(pose_, planner_id_, path)) {
        return false;
    }
    automsgs::msgs::nav_msgs::Path smoothed;
    if (nav_->SmoothPath(path, smoother_id_, 1.0, true, smoothed)) {
        path = std::move(smoothed);
    }
    automsgs::actions::FollowPathAction::Goal follow;
    *follow.mutable_path() = path;
    follow.set_controller_id(controller_id_);
    return nav_->follow_session().Begin(nav_->follow_path_client(), follow);
}

bool DockNavigator::Begin(
    const ::automsgs::msgs::geometry_msgs::PoseStamped& pose,
    std::chrono::milliseconds timeout) {
    if (!nav_) {
        throw FailedToStage("no navigation client for staging");
    }
    Cancel();
    pose_ = pose;
    deadline_ = std::chrono::steady_clock::now() + timeout;
    recursed_ = false;
    if (!LaunchFollow()) {
        throw FailedToStage("failed to plan/follow to staging pose");
    }
    sent_ = true;
    status_ = Status::Running;
    return true;
}

DockNavigator::Status DockNavigator::Tick(bool canceled) {
    if (status_ != Status::Running) {
        return status_;
    }
    if (canceled) {
        Cancel();
        status_ = Status::Canceled;
        return status_;
    }
    if (std::chrono::steady_clock::now() > deadline_) {
        Cancel();
        throw FailedToStage("Navigation request to staging pose timed out.");
    }
    const auto st = nav_->follow_session().Tick(
        []() { return false; },
        [](const automsgs::actions::FollowPathAction::Result&) {},
        [](int, const std::string&) {});
    if (st == BT::NodeStatus::SUCCESS) {
        status_ = Status::Succeeded;
        return status_;
    }
    if (st != BT::NodeStatus::FAILURE) {
        return status_;
    }
    AWARN << "Navigation request to staging pose failed.";
    if (!recursed_) {
        recursed_ = true;
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline_ - std::chrono::steady_clock::now());
        if (remaining.count() > 0 && LaunchFollow()) {
            sent_ = true;
            return status_;
        }
    }
    throw FailedToStage("Navigation request to staging pose failed.");
}

void DockNavigator::Cancel() {
    if (nav_ && sent_) {
        nav_->follow_session().Cancel(nav_->follow_path_client());
        nav_->CancelActiveMotion();
    }
    sent_ = false;
    if (status_ == Status::Running) {
        status_ = Status::Idle;
    }
}

}  // namespace charging
}  // namespace task
}  // namespace autonomy
