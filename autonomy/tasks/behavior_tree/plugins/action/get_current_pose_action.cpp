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

#include "autonomy/tasks/behavior_tree/plugins/action/get_current_pose_action.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/utils/robot_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

GetCurrentPoseAction::GetCurrentPoseAction(const std::string& name,
                                           const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(name, conf) {
    node_ = config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
    tf_ =
        config().blackboard->get<std::shared_ptr<autonomy::transform::Buffer>>(
            "tf_buffer");
    getInput("transform_tolerance", transform_tolerance_);
    global_frame_ =
        DeconflictPortAndParamFrame<std::string>(node_, "global_frame", this);
    robot_base_frame_ = DeconflictPortAndParamFrame<std::string>(
        node_, "robot_base_frame", this);
}

BT::NodeStatus GetCurrentPoseAction::tick() {
    setStatus(BT::NodeStatus::RUNNING);
    commsgs::geometry_msgs::PoseStamped current_pose;

    if (!autonomy::tasks::utils::getCurrentPose(
            current_pose, tf_, global_frame_, robot_base_frame_,
            transform_tolerance_)) {
        AWARN << "Current robot pose is not available.";
        return BT::NodeStatus::FAILURE;
    }

    setOutput("current_pose", current_pose);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::plugins::action::GetCurrentPoseAction>(
        "GetCurrentPose");
}
