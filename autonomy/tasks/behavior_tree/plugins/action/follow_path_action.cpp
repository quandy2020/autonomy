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

#include "autonomy/tasks/behavior_tree/plugins/action/follow_path_action.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller_server.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

namespace {

using ErrorCode = proto::FollowPathErrorCode;
using TickResult = control::ControllerServer::FollowPathTickResult;

}  // namespace

FollowPathAction::FollowPathAction(const std::string& xml_tag_name,
                                   const BT::NodeConfiguration& conf)
    : BtStatefulActionNode(xml_tag_name, conf) {}

void FollowPathAction::setFailure(int32_t code, const std::string& msg) {
    setOutput("error_code_id", code);
    setOutput("error_msg", msg);
}

void FollowPathAction::maybeUpdatePathFromPorts() {
    commsgs::planning_msgs::Path new_path;
    if (getInput("path", new_path) && !new_path.poses.empty()) {
        const bool path_changed =
            path_.poses.size() != new_path.poses.size() ||
            path_.poses.empty() ||
            path_.poses.back().pose.position.x !=
                new_path.poses.back().pose.position.x ||
            path_.poses.back().pose.position.y !=
                new_path.poses.back().pose.position.y;
        if (path_changed) {
            path_ = std::move(new_path);
            auto ctx = taskContext();
            if (ctx && ctx->controller && follow_started_) {
                ctx->controller->BeginFollowPath(path_, controller_id_,
                                                 goal_checker_id_,
                                                 progress_checker_id_);
            }
        }
    }

    std::string new_controller_id;
    if (getInput("controller_id", new_controller_id) &&
        new_controller_id != controller_id_) {
        controller_id_ = new_controller_id;
    }
    std::string new_goal_checker_id;
    if (getInput("goal_checker_id", new_goal_checker_id) &&
        new_goal_checker_id != goal_checker_id_) {
        goal_checker_id_ = new_goal_checker_id;
    }
    std::string new_progress_checker_id;
    if (getInput("progress_checker_id", new_progress_checker_id) &&
        new_progress_checker_id != progress_checker_id_) {
        progress_checker_id_ = new_progress_checker_id;
    }
}

BT::NodeStatus FollowPathAction::onStart() {
    follow_started_ = false;
    auto ctx = taskContext();
    if (!ctx || !ctx->controller) {
        setFailure(static_cast<int32_t>(ErrorCode::FOLLOW_PATH_ERROR_UNKNOWN),
                   "TaskContext or ControllerServer is not available.");
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("path", path_) || path_.poses.empty()) {
        setFailure(static_cast<int32_t>(ErrorCode::FOLLOW_PATH_ERROR_INVALID_PATH),
                   "Missing or empty path input.");
        return BT::NodeStatus::FAILURE;
    }

    controller_id_ = ctx->selected_controller_id;
    goal_checker_id_ = ctx->selected_goal_checker_id;
    progress_checker_id_ = ctx->selected_progress_checker_id;
    getInput("controller_id", controller_id_);
    getInput("goal_checker_id", goal_checker_id_);
    getInput("progress_checker_id", progress_checker_id_);

    if (!ctx->controller->BeginFollowPath(path_, controller_id_,
                                          goal_checker_id_,
                                          progress_checker_id_)) {
        setFailure(static_cast<int32_t>(
                       ErrorCode::FOLLOW_PATH_ERROR_INVALID_CONTROLLER),
                   "Failed to start follow path on controller.");
        return BT::NodeStatus::FAILURE;
    }

    follow_started_ = true;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowPathAction::onRunning() {
    maybeUpdatePathFromPorts();

    auto ctx = taskContext();
    if (!ctx || !ctx->controller) {
        setFailure(static_cast<int32_t>(ErrorCode::FOLLOW_PATH_ERROR_UNKNOWN),
                   "ControllerServer is not available.");
        return BT::NodeStatus::FAILURE;
    }

    const TickResult tick =
        ctx->controller->TickFollowPath(ctx->CancelChecker());
    switch (tick) {
        case TickResult::Running:
            return BT::NodeStatus::RUNNING;
        case TickResult::Succeeded:
            setOutput("error_code_id",
                      static_cast<int32_t>(ErrorCode::FOLLOW_PATH_ERROR_NONE));
            setOutput("error_msg", std::string(""));
            follow_started_ = false;
            return BT::NodeStatus::SUCCESS;
        case TickResult::Cancelled:
            setOutput("error_code_id",
                      static_cast<int32_t>(ErrorCode::FOLLOW_PATH_ERROR_NONE));
            setOutput("error_msg", std::string(""));
            follow_started_ = false;
            return BT::NodeStatus::SUCCESS;
        case TickResult::Failed:
        default:
            setFailure(static_cast<int32_t>(ErrorCode::FOLLOW_PATH_ERROR_UNKNOWN),
                       "Follow path failed.");
            follow_started_ = false;
            return BT::NodeStatus::FAILURE;
    }
}

void FollowPathAction::onHalted() {
    auto ctx = taskContext();
    if (ctx && ctx->controller) {
        ctx->controller->EndFollowPath();
    }
    follow_started_ = false;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    BT::NodeBuilder builder = [](const std::string& name,
                                 const BT::NodeConfiguration& config) {
        return std::make_unique<
            autonomy::tasks::behavior_tree::plugins::action::FollowPathAction>(
            name, config);
    };

    factory.registerBuilder<
        autonomy::tasks::behavior_tree::plugins::action::FollowPathAction>(
        "FollowPath", builder);
}
