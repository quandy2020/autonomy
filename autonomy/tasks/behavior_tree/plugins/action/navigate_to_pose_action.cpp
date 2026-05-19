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

#include "autonomy/tasks/behavior_tree/plugins/action/navigate_to_pose_action.hpp"

#include <set>
#include <string>
#include <vector>

#include "autonomy/common/log.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

NavigateToPoseAction::NavigateToPoseAction(const std::string& xml_tag_name,
                                           const std::string& action_name,
                                           const BT::NodeConfiguration& conf)
    : BtActionNode<Action>(xml_tag_name, action_name, conf) {}

void NavigateToPoseAction::on_tick() {
    commsgs::geometry_msgs::PoseStamped goal_pose;
    if (getInput("goal", goal_pose)) {
        auto* proto_pose = goal_.mutable_pose();
        *proto_pose = commsgs::geometry_msgs::ToProto(goal_pose);
    } else {
        AERROR << "NavigateToPoseAction: goal not provided";
        return;
    }

    std::string behavior_tree;
    if (getInput("behavior_tree", behavior_tree)) {
        goal_.set_behavior_tree(behavior_tree);
    }
}

BT::NodeStatus NavigateToPoseAction::on_success() {
    setOutput("error_code_id",
              static_cast<int32_t>(
                  proto::NavigateToPoseErrorCode::NAVIGATE_TO_POSE_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus NavigateToPoseAction::on_aborted() {
    if (result_.result) {
        setOutput("error_code_id",
                  static_cast<int32_t>(result_.result->error_code()));
        setOutput("error_msg", result_.result->error_msg());
    } else {
        setOutput("error_code_id",
                  static_cast<int32_t>(proto::NavigateToPoseErrorCode::
                                           NAVIGATE_TO_POSE_ERROR_UNKNOWN));
        setOutput("error_msg", std::string("Action aborted"));
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus NavigateToPoseAction::on_cancelled() {
    // Set empty error code, action was cancelled
    setOutput("error_code_id",
              static_cast<int32_t>(
                  proto::NavigateToPoseErrorCode::NAVIGATE_TO_POSE_ERROR_NONE));
    setOutput("error_msg", std::string(""));
    return BT::NodeStatus::SUCCESS;
}

void NavigateToPoseAction::on_timeout() {
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
                                    action::NavigateToPoseAction>(
            name, "navigate_to_pose", config);
    };

    factory.registerBuilder<
        autonomy::tasks::behavior_tree::plugins::action::NavigateToPoseAction>(
        "NavigateToPose", builder);
}
