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

#include <chrono>
#include <string>
#include <vector>

#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/json_export.h"

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace decorator {

/**
 * @brief A BT::DecoratorNode that ticks its child if the goal was updated
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class GoalUpdatedController : public BT::DecoratorNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::GoalUpdatedController
     * @param name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    GoalUpdatedController(const std::string& name, const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts() {
        // Register JSON definitions for the types used in the ports
        BT::RegisterJsonDefinition<commsgs::geometry_msgs::PoseStamped>();
        BT::RegisterJsonDefinition<commsgs::planning_msgs::Goals>();

        return {
            BT::InputPort<commsgs::planning_msgs::Goals>("goals", "Vector of navigation goals"),
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>("goal", "Navigation goal"),
        };
    }

private:
    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    bool goal_was_updated_;
    commsgs::geometry_msgs::PoseStamped goal_;
    commsgs::planning_msgs::Goals goals_;
};

}  // namespace decorator
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy