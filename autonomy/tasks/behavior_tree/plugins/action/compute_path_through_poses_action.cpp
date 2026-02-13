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

#include "autonomy/tasks/behavior_tree/plugins/action/compute_path_through_poses_action.hpp"

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

ComputePathThroughPosesAction::ComputePathThroughPosesAction(const std::string& xml_tag_name,
                                                             const std::string& action_name,
                                                             const BT::NodeConfiguration& conf)
    : BtActionNode<Action>(xml_tag_name, action_name, conf) {}

void ComputePathThroughPosesAction::on_tick() {
    commsgs::planning_msgs::Goals goals;
    if (getInput("goals", goals)) {
        // Convert commsgs Goals to proto Goals
        auto* proto_goals = goal_.mutable_goals();
        *proto_goals->mutable_header() = commsgs::std_msgs::ToProto(goals.header);
        for (const auto& goal : goals.goals) {
            auto* proto_goal = proto_goals->add_goals();
            *proto_goal = commsgs::geometry_msgs::ToProto(goal);
        }
    }

    std::string planner_id;
    if (getInput("planner_id", planner_id)) {
        goal_.set_planner_id(planner_id);
    }

    commsgs::geometry_msgs::PoseStamped start_pose;
    if (getInput("start", start_pose)) {
        auto* proto_start = goal_.mutable_start();
        *proto_start = commsgs::geometry_msgs::ToProto(start_pose);
        goal_.set_use_start(true);
    }
}

BT::NodeStatus ComputePathThroughPosesAction::on_success() {
    if (result_.result) {
        commsgs::planning_msgs::Path path;
        // Convert proto Path to commsgs Path
        path.header = commsgs::std_msgs::FromProto(result_.result->path().header());
        for (const auto& pose_proto : result_.result->path().poses()) {
            path.poses.push_back(commsgs::geometry_msgs::FromProto(pose_proto));
        }
        setOutput("path", path);
    }
    // Set empty error code, action was successful
    setOutput("error_code_id",
              static_cast<int32_t>(proto::ComputePathThroughPosesErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathThroughPosesAction::on_aborted() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("path", empty_path);
    if (result_.result) {
        setOutput("error_code_id", static_cast<int32_t>(result_.result->error_code()));
        setOutput("error_msg", result_.result->error_msg());
    } else {
        setOutput(
            "error_code_id",
            static_cast<int32_t>(proto::ComputePathThroughPosesErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_UNKNOWN));
        setOutput("error_msg", std::string("Action aborted"));
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathThroughPosesAction::on_cancelled() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("path", empty_path);
    // Set empty error code, action was cancelled
    setOutput("error_code_id",
              static_cast<int32_t>(proto::ComputePathThroughPosesErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

void ComputePathThroughPosesAction::on_timeout() {
    setOutput("error_code_id",
              static_cast<int32_t>(proto::ComputePathThroughPosesErrorCode::COMPUTE_PATH_THROUGH_POSES_ERROR_TIMEOUT));
    setOutput("error_msg", std::string("Behavior Tree action client timed out waiting."));
}

void ComputePathThroughPosesAction::halt() {
    commsgs::planning_msgs::Path empty_path;
    setOutput("path", empty_path);
    // DO NOT reset "error_code_id" output port, we want to read it later
    // DO NOT reset "error_msg" output port, we want to read it later
    BtActionNode<Action>::halt();
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<autonomy::tasks::behavior_tree::plugins::action::ComputePathThroughPosesAction>(
            name, "compute_path_through_poses", config);
    };

    factory.registerBuilder<autonomy::tasks::behavior_tree::plugins::action::ComputePathThroughPosesAction>(
        "ComputePathThroughPoses", builder);
}
