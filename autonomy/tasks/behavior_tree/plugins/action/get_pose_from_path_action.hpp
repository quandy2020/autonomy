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

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief A BT::ActionNode that gets a pose from a path at a specific index
 */
class GetPoseFromPathAction : public BT::ActionNodeBase
{
public:
    /**
     * @brief A constructor for
     * autonomy::tasks::behavior_tree::plugins::action::GetPoseFromPathAction
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    GetPoseFromPathAction(const std::string& xml_tag_name,
                          const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<commsgs::planning_msgs::Path>("path", "Input path"),
            BT::InputPort<size_t>("index", 0,
                                  "Index of the pose to extract from path"),
            BT::OutputPort<commsgs::geometry_msgs::PoseStamped>(
                "pose", "Extracted pose from path"),
        };
    }

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Function to halt the node
     */
    void halt() override {}
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
