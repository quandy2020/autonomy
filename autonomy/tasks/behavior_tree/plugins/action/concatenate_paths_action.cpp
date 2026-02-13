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

#include "autonomy/tasks/behavior_tree/plugins/action/concatenate_paths_action.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

ConcatenatePaths::ConcatenatePaths(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(name, conf) {}

BT::NodeStatus ConcatenatePaths::tick() {
    setStatus(BT::NodeStatus::RUNNING);

    commsgs::planning_msgs::Path input_path1, input_path2;
    getInput("input_path1", input_path1);
    getInput("input_path2", input_path2);

    if (input_path1.poses.empty() && input_path2.poses.empty()) {
        AERROR << "No input paths provided to concatenate. Both paths are empty.";
        return BT::NodeStatus::FAILURE;
    }

    commsgs::planning_msgs::Path output_path;
    output_path = input_path1;
    if (input_path1.header.stamp.sec != 0 || input_path1.header.stamp.nanosec != 0 ||
        !input_path1.header.frame_id.empty()) {
        output_path.header = input_path1.header;
    } else if (input_path2.header.stamp.sec != 0 || input_path2.header.stamp.nanosec != 0 ||
               !input_path2.header.frame_id.empty()) {
        output_path.header = input_path2.header;
    }

    output_path.poses.insert(output_path.poses.end(), input_path2.poses.begin(), input_path2.poses.end());

    setOutput("output_path", output_path);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::action::ConcatenatePaths>("ConcatenatePaths");
}
