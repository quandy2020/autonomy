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

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief The ProgressCheckerSelector behavior is used to switch the progress
 * checker that will be used by the controller server
 * @note This is an Asynchronous node. It will re-initialize when halted.
 */
class ProgressCheckerSelector : public BT::SyncActionNode
{
public:
    /**
     * @brief A constructor for
     * autonomy::tasks::behavior_tree::plugins::action::ProgressCheckerSelector
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    ProgressCheckerSelector(const std::string& xml_tag_name,
                            const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("topic_name", "",
                                       "Topic name to select progress checker"),
            BT::InputPort<std::string>("default_progress_checker", "",
                                       "Default progress checker name"),
            BT::OutputPort<std::string>("selected_progress_checker",
                                        "Selected progress checker name"),
        };
    }

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
