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

#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

GetCurrentPoseAction::GetCurrentPoseAction(const std::string& xml_tag_name,
                                           const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf) {}

BT::NodeStatus GetCurrentPoseAction::tick() {
    // TODO: Implement get current pose behavior
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
