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

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief A BT::ActionNode that executes a drive on heading behavior
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class DriveOnHeadingAction : public BT::ActionNodeBase
{
public:
    /**
     * @brief A constructor for
     * autonomy::tasks::behavior_tree::plugins::action::DriveOnHeadingAction
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    DriveOnHeadingAction(const std::string& xml_tag_name,
                         const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("distance", 0.5,
                                  "Distance to drive in meters"),
            BT::InputPort<double>("speed", 0.2,
                                  "Speed at which to drive in m/s"),
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
