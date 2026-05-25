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

#include <set>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief The PlannerSelector behavior is used to switch the planner
 * that will be used by the planner server. Reads default_planner / blackboard;
 * topic_name is kept for BT XML compatibility (no external subscription).
 * Usually runs before ComputePathToPose. The selected_planner
 * output port is passed to planner_id input port of the ComputePathToPoseAction
 * @note This is an Asynchronous node. It will re-initialize when halted.
 */
class PlannerSelector : public BT::SyncActionNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::PlannerSelector
     *
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf  BT node configuration
     */
    PlannerSelector(const std::string& xml_tag_name,
                    const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>(
                    "default_planner",
                    "the default planner to use if there is not "
                    "any external topic message received."),

                BT::InputPort<std::string>(
                    "topic_name", "planner_selector",
                    "the input topic name to select the planner"),

                BT::OutputPort<std::string>(
                    "selected_planner", "Selected planner by subscription")};
    }

private:
    /**
     * @brief Function to read parameters and initialize class variables
     */
    BT::NodeStatus tick() override;

    std::string last_selected_planner_;
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy