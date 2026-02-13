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

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/json_export.h"

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "autonomy/tasks/navigator/proto/msg.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

/**
 * @brief A BT::ConditionNode that returns SUCCESS every time a specified
 * time period passes and FAILURE otherwise
 */
class PathExpiringTimerCondition : public BT::ConditionNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::PathExpiringTimerCondition
     * @param condition_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    PathExpiringTimerCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

    PathExpiringTimerCondition() = delete;

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts() {
        // Register JSON definitions for the types used in the ports
        BT::RegisterJsonDefinition<commsgs::planning_msgs::Path>();

        return {BT::InputPort<double>("seconds", 1.0, "Seconds"), BT::InputPort<commsgs::planning_msgs::Path>("path")};
    }

private:
    std::shared_ptr<::autolink::Node> node_;
    commsgs::builtin_interfaces::Time start_;
    commsgs::planning_msgs::Path prev_path_;
    double period_;
    bool first_time_;
};

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy