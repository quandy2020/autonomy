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

#include "autonomy/tasks/behavior_tree/plugins/action/compute_route_action.hpp"

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

ComputeRouteAction::ComputeRouteAction(const std::string& xml_tag_name, const std::string& action_name,
                                       const BT::NodeConfiguration& conf)
    : BtActionNode<Action>(xml_tag_name, action_name, conf) {}

void ComputeRouteAction::on_tick() {
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

BT::NodeStatus ComputeRouteAction::on_success() {
    if (result_.result) {
        if (result_.result->has_path()) {
            const auto& proto_path = result_.result->path();
            commsgs::planning_msgs::Path path;
            path.header = commsgs::std_msgs::FromProto(proto_path.header());
            for (const auto& proto_pose : proto_path.poses()) {
                path.poses.push_back(commsgs::geometry_msgs::FromProto(proto_pose));
            }
            setOutput("path", path);
        } else {
            commsgs::planning_msgs::Path empty_path;
            setOutput("path", empty_path);
        }

        if (result_.result->has_planning_time()) {
            const auto& proto_duration = result_.result->planning_time();
            commsgs::builtin_interfaces::Duration duration = commsgs::builtin_interfaces::FromProto(proto_duration);
            setOutput("planning_time", duration);
        } else {
            commsgs::builtin_interfaces::Duration empty_duration;
            setOutput("planning_time", empty_duration);
        }

        if (result_.result->has_route()) {
            setOutput("route", result_.result->route());
        } else {
            proto::Route empty_route;
            setOutput("route", empty_route);
        }
    } else {
        commsgs::planning_msgs::Path empty_path;
        setOutput("path", empty_path);
        commsgs::builtin_interfaces::Duration empty_duration;
        setOutput("planning_time", empty_duration);
        proto::Route empty_route;
        setOutput("route", empty_route);
    }

    setOutput("error_code_id", static_cast<int32_t>(proto::ComputeRouteActionErrorCode::COMPUTE_ROUTE_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeRouteAction::on_aborted() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("path", empty_path);
    commsgs::builtin_interfaces::Duration empty_duration;
    setOutput("planning_time", empty_duration);
    proto::Route empty_route;
    setOutput("route", empty_route);

    if (result_.result) {
        setOutput("error_code_id", static_cast<int32_t>(result_.result->error_code()));
        setOutput("error_msg", result_.result->error_msg());
    } else {
        setOutput("error_code_id",
                  static_cast<int32_t>(proto::ComputeRouteActionErrorCode::COMPUTE_ROUTE_ERROR_UNKNOWN));
        setOutput("error_msg", std::string("Unknown error"));
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeRouteAction::on_cancelled() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("path", empty_path);
    commsgs::builtin_interfaces::Duration empty_duration;
    setOutput("planning_time", empty_duration);
    proto::Route empty_route;
    setOutput("route", empty_route);

    setOutput("error_code_id", static_cast<int32_t>(proto::ComputeRouteActionErrorCode::COMPUTE_ROUTE_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

void ComputeRouteAction::on_timeout() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("path", empty_path);
    commsgs::builtin_interfaces::Duration empty_duration;
    setOutput("planning_time", empty_duration);
    proto::Route empty_route;
    setOutput("route", empty_route);

    setOutput("error_code_id", static_cast<int32_t>(proto::ComputeRouteActionErrorCode::COMPUTE_ROUTE_ERROR_TIMEOUT));
    setOutput("error_msg", std::string("Behavior Tree action client timed out waiting."));
}

void ComputeRouteAction::halt() {
    resetPorts();
    BtActionNode<Action>::halt();
}

void ComputeRouteAction::resetPorts() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("path", empty_path);
    commsgs::builtin_interfaces::Duration empty_duration;
    setOutput("planning_time", empty_duration);
    proto::Route empty_route;
    setOutput("route", empty_route);
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<autonomy::tasks::behavior_tree::plugins::action::ComputeRouteAction>(
            name, "compute_route", config);
    };

    factory.registerBuilder<autonomy::tasks::behavior_tree::plugins::action::ComputeRouteAction>("ComputeRoute",
                                                                                                 builder);
}
