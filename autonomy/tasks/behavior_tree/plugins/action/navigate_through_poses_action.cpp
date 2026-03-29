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

#include "autonomy/tasks/behavior_tree/plugins/action/navigate_through_poses_action.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

NavigateThroughPosesAction::NavigateThroughPosesAction(
    const std::string& xml_tag_name, const std::string& action_name,
    const BT::NodeConfiguration& conf)
    : BtActionNode<proto::NavigateToPoseAction>(xml_tag_name, action_name,
                                                conf) {}

void NavigateThroughPosesAction::on_tick() {
    commsgs::geometry_msgs::PoseStamped pose;
    if (!getInput("goals", pose)) {
        AERROR << "NavigateThroughPosesAction: goal not provided";
        return;
    }
    *goal_.mutable_pose() = commsgs::geometry_msgs::ToProto(pose);

    std::string behavior_tree;
    if (getInput("behavior_tree", behavior_tree)) {
        goal_.set_behavior_tree(behavior_tree);
    }
}

BT::NodeStatus NavigateThroughPosesAction::on_success() {
    setOutput("error_code_id",
              static_cast<int32_t>(
                  proto::NavigateToPoseErrorCode::NAVIGATE_TO_POSE_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus NavigateThroughPosesAction::on_aborted() {
    if (result_.result) {
        setOutput("error_code_id",
                  static_cast<int32_t>(result_.result->error_code()));
        setOutput("error_msg", result_.result->error_msg());
    } else {
        setOutput("error_code_id",
                  static_cast<int32_t>(proto::NavigateToPoseErrorCode::
                                           NAVIGATE_TO_POSE_ERROR_UNKNOWN));
        setOutput("error_msg", std::string("Unknown error"));
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus NavigateThroughPosesAction::on_cancelled() {
    setOutput("error_code_id",
              static_cast<int32_t>(
                  proto::NavigateToPoseErrorCode::NAVIGATE_TO_POSE_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

void NavigateThroughPosesAction::on_timeout() {
    setOutput(
        "error_code_id",
        static_cast<int32_t>(
            proto::NavigateToPoseErrorCode::NAVIGATE_TO_POSE_ERROR_TIMEOUT));
    setOutput("error_msg",
              std::string("Behavior Tree action client timed out waiting."));
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
                                    action::NavigateThroughPosesAction>(
            name, "navigate_through_poses", config);
    };

    factory.registerBuilder<autonomy::tasks::behavior_tree::plugins::action::
                                NavigateThroughPosesAction>(
        "NavigateThroughPoses", builder);
}
