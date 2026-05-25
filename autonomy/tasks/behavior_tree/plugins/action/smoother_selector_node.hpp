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

/** @brief Select smoother id from blackboard default or BT input ports. */
class SmootherSelector : public BT::SyncActionNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::SmootherSelector
     *
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf  BT node configuration
     */
    SmootherSelector(const std::string& xml_tag_name,
                     const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>(
                    "default_smoother",
                    "the default smoother to use if there is not any external "
                    "topic message received."),

                BT::InputPort<std::string>(
                    "topic_name", "smoother_selector",
                    "the input topic name to select the smoother"),

                BT::OutputPort<std::string>("selected_smoother",
                                           "Selected smoother id")};
    }

private:
    void readDefaultFromPorts();

    BT::NodeStatus tick() override;

    std::string last_selected_smoother_;
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
