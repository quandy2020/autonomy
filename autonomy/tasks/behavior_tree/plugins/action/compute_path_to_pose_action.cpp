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

#include "autonomy/tasks/behavior_tree/plugins/action/compute_path_to_pose_action.hpp"

#include "autonomy/common/logging.hpp"

#include <set>
#include <string>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

ComputePathToPoseAction::ComputePathToPoseAction(
    const std::string& xml_tag_name, const std::string& action_name,
    const BT::NodeConfiguration& conf)
    : BtActionNode<Action>(xml_tag_name, action_name, conf) {}

void ComputePathToPoseAction::on_tick() {
    commsgs::geometry_msgs::PoseStamped goal_pose;
    if (getInput("goal", goal_pose)) {
        auto* proto_goal = goal_.mutable_goal();
        *proto_goal = commsgs::geometry_msgs::ToProto(goal_pose);
    }

    std::string planner_id;
    if (getInput("planner_id", planner_id)) {
        goal_.set_planner_id(planner_id);
    }

    // if "use_start" is provided try to enforce it (true or false), but we
    // cannot enforce true if start is not provided
    bool use_start = false;
    if (getInput("use_start", use_start)) {
        commsgs::geometry_msgs::PoseStamped start_pose;
        if (use_start && !getInput("start", start_pose)) {
            // in case we don't have a "start" pose
            use_start = false;
            AERROR
                << "use_start is set to true but no start pose was provided, "
                   "falling back to default behavior, i.e. using the current "
                   "robot pose";
        } else if (use_start) {
            auto* proto_start = goal_.mutable_start();
            *proto_start = commsgs::geometry_msgs::ToProto(start_pose);
        }
        goal_.set_use_start(use_start);
    } else {
        // else if "use_start" is not provided, but "start" is, then use it in
        // order to not change the legacy behavior
        commsgs::geometry_msgs::PoseStamped start_pose;
        if (getInput("start", start_pose)) {
            goal_.set_use_start(true);
            auto* proto_start = goal_.mutable_start();
            *proto_start = commsgs::geometry_msgs::ToProto(start_pose);
        }
    }
}

BT::NodeStatus ComputePathToPoseAction::on_success() {
    if (result_.result && result_.result->has_path()) {
        // Convert proto Path to commsgs Path
        commsgs::planning_msgs::Path path;
        const auto& proto_path = result_.result->path();
        path.header = commsgs::std_msgs::FromProto(proto_path.header());
        for (const auto& proto_pose : proto_path.poses()) {
            path.poses.push_back(commsgs::geometry_msgs::FromProto(proto_pose));
        }
        setOutput("path", path);
    } else {
        commsgs::planning_msgs::Path empty_path;
        setOutput("path", empty_path);
    }
    // Set empty error code, action was successful
    setOutput(
        "error_code_id",
        static_cast<int32_t>(
            autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::
                COMPUTE_PATH_TO_POSE_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathToPoseAction::on_aborted() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("path", empty_path);
    if (result_.result) {
        setOutput("error_code_id",
                  static_cast<int32_t>(result_.result->error_code()));
        setOutput("error_msg", result_.result->error_msg());
    } else {
        setOutput(
            "error_code_id",
            static_cast<int32_t>(autonomy::tasks::behavior_tree::proto::
                                     ComputePathToPoseErrorCode::
                                         COMPUTE_PATH_TO_POSE_ERROR_UNKNOWN));
        setOutput("error_msg", std::string("Unknown error"));
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathToPoseAction::on_cancelled() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("path", empty_path);
    // Set empty error code, action was cancelled
    setOutput(
        "error_code_id",
        static_cast<int32_t>(
            autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::
                COMPUTE_PATH_TO_POSE_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

void ComputePathToPoseAction::on_timeout() {
    setOutput(
        "error_code_id",
        static_cast<int32_t>(
            autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::
                COMPUTE_PATH_TO_POSE_ERROR_TIMEOUT));
    setOutput("error_msg",
              std::string("Behavior Tree action client timed out waiting."));
}

void ComputePathToPoseAction::halt() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("path", empty_path);
    // DO NOT reset "error_code_id" output port, we want to read it later
    // DO NOT reset "error_msg" output port, we want to read it later
    BtActionNode::halt();
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
        return std::make_unique<autonomy::tasks::behavior_tree::plugins::
                                    action::ComputePathToPoseAction>(
            name, "compute_path_to_pose", config);
    };

    factory.registerBuilder<autonomy::tasks::behavior_tree::plugins::action::
                                ComputePathToPoseAction>("ComputePathToPose",
                                                         builder);
}
