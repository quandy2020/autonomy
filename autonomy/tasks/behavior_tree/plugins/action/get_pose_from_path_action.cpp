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

#include "autonomy/tasks/behavior_tree/plugins/action/get_pose_from_path_action.hpp"

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

GetPoseFromPathAction::GetPoseFromPathAction(const std::string& xml_tag_name,
                                             const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(xml_tag_name, conf) {}

BT::NodeStatus GetPoseFromPathAction::tick() {
    commsgs::planning_msgs::Path path;
    size_t index = 0;

    if (!getInput("path", path)) {
        return BT::NodeStatus::FAILURE;
    }

    getInput("index", index);

    if (index >= path.poses.size()) {
        return BT::NodeStatus::FAILURE;
    }

    setOutput("pose", path.poses[index]);
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
        autonomy::tasks::behavior_tree::plugins::action::GetPoseFromPathAction>(
        "GetPoseFromPath");
}
