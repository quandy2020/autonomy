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

#pragma once

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/json_export.h"

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

class GetPoseFromPath : public BT::ActionNodeBase
{
public:
    GetPoseFromPath(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts() {
        // Register JSON definitions for the types used in the ports
        BT::RegisterJsonDefinition<commsgs::geometry_msgs::PoseStamped>();
        BT::RegisterJsonDefinition<commsgs::planning_msgs::Path>();

        return {
            BT::InputPort<commsgs::planning_msgs::Path>("path", "Path to extract pose from"),
            BT::OutputPort<commsgs::geometry_msgs::PoseStamped>("pose", "Stamped Extracted Pose"),
            BT::InputPort<int>("index", 0, "Index of pose to extract from. -1 is end of list"),
        };
    }

private:
    void halt() override {}
    BT::NodeStatus tick() override;
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
