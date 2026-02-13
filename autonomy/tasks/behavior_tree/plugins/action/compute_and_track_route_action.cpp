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

#include "autonomy/tasks/behavior_tree/plugins/action/compute_and_track_route_action.hpp"

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

ComputeAndTrackRouteAction::ComputeAndTrackRouteAction(const std::string& xml_tag_name, const std::string& action_name,
                                                       const BT::NodeConfiguration& conf)
    : BtActionNode<Action>(xml_tag_name, action_name, conf) {}

void ComputeAndTrackRouteAction::on_tick() {
    bool use_poses = false, use_start = false;
    getInput("use_poses", use_poses);
    goal_.set_use_poses(use_poses);

    if (use_poses) {
        commsgs::geometry_msgs::PoseStamped goal;
        getInput("goal", goal);
        *goal_.mutable_goal() = commsgs::geometry_msgs::ToProto(goal);

        getInput("use_start", use_start);
        goal_.set_use_start(use_start);
        if (use_start) {
            commsgs::geometry_msgs::PoseStamped start;
            getInput("start", start);
            *goal_.mutable_start() = commsgs::geometry_msgs::ToProto(start);
        }
    } else {
        uint32_t start_id = 0;
        uint32_t goal_id = 0;
        getInput("start_id", start_id);
        getInput("goal_id", goal_id);
        goal_.set_start_id(start_id);
        goal_.set_goal_id(goal_id);
        goal_.set_use_start(false);
    }
}

BT::NodeStatus ComputeAndTrackRouteAction::on_success() {
    if (result_.result && result_.result->has_execution_duration()) {
        const auto& proto_duration = result_.result->execution_duration();
        commsgs::builtin_interfaces::Duration duration = commsgs::builtin_interfaces::FromProto(proto_duration);
        setOutput("execution_duration", duration);
    } else {
        commsgs::builtin_interfaces::Duration empty_duration;
        setOutput("execution_duration", empty_duration);
    }

    setOutput("error_code_id",
              static_cast<int32_t>(proto::ComputeAndTrackRouteErrorCode::COMPUTE_AND_TRACK_ROUTE_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeAndTrackRouteAction::on_aborted() {
    commsgs::builtin_interfaces::Duration empty_duration;
    setOutput("execution_duration", empty_duration);

    if (result_.result) {
        setOutput("error_code_id", static_cast<int32_t>(result_.result->error_code()));
        setOutput("error_msg", result_.result->error_msg());
    } else {
        setOutput("error_code_id",
                  static_cast<int32_t>(proto::ComputeAndTrackRouteErrorCode::COMPUTE_AND_TRACK_ROUTE_ERROR_UNKNOWN));
        setOutput("error_msg", std::string("Unknown error"));
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeAndTrackRouteAction::on_cancelled() {
    commsgs::builtin_interfaces::Duration empty_duration;
    setOutput("execution_duration", empty_duration);

    setOutput("error_code_id",
              static_cast<int32_t>(proto::ComputeAndTrackRouteErrorCode::COMPUTE_AND_TRACK_ROUTE_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

void ComputeAndTrackRouteAction::on_timeout() {
    setOutput("error_code_id",
              static_cast<int32_t>(proto::ComputeAndTrackRouteErrorCode::COMPUTE_AND_TRACK_ROUTE_ERROR_TIMEOUT));
    setOutput("error_msg", std::string("Behavior Tree action client timed out waiting."));
}

void ComputeAndTrackRouteAction::on_wait_for_result(std::shared_ptr<const Action::Feedback> /*feedback*/) {
    // Check for request updates to the goal
    bool use_poses = false, use_start = false;
    getInput("use_start", use_start);
    getInput("use_poses", use_poses);

    if (goal_.use_poses() != use_poses) {
        goal_updated_ = true;
    }

    if (use_poses) {
        commsgs::geometry_msgs::PoseStamped goal;
        getInput("goal", goal);
        auto proto_goal = commsgs::geometry_msgs::ToProto(goal);
        // Compare proto messages
        if (!goal_.has_goal() || goal_.goal().pose().position().x() != proto_goal.pose().position().x() ||
            goal_.goal().pose().position().y() != proto_goal.pose().position().y()) {
            goal_updated_ = true;
        }

        if (goal_.use_start() != use_start) {
            goal_updated_ = true;
        }
        if (use_start) {
            commsgs::geometry_msgs::PoseStamped start;
            getInput("start", start);
            auto proto_start = commsgs::geometry_msgs::ToProto(start);
            if (!goal_.has_start() || goal_.start().pose().position().x() != proto_start.pose().position().x() ||
                goal_.start().pose().position().y() != proto_start.pose().position().y()) {
                goal_updated_ = true;
            }
        }
    } else {
        // Check if the start and goal IDs have changed
        uint32_t start_id = 0;
        uint32_t goal_id = 0;
        getInput("start_id", start_id);
        getInput("goal_id", goal_id);
        if (goal_.start_id() != start_id) {
            goal_updated_ = true;
        }
        if (goal_.goal_id() != goal_id) {
            goal_updated_ = true;
        }
    }

    // If we're updating the request, we need to fully update the goal
    // Easier to call on_tick() again than to duplicate the code
    if (goal_updated_) {
        on_tick();
    }
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<autonomy::tasks::behavior_tree::plugins::action::ComputeAndTrackRouteAction>(
            name, "compute_and_track_route", config);
    };

    factory.registerBuilder<autonomy::tasks::behavior_tree::plugins::action::ComputeAndTrackRouteAction>(
        "ComputeAndTrackRoute", builder);
}
