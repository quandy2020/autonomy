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

#include <memory>
#include <string>

#include "autonomy/tasks/navigator/proto/srv.pb.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/json_export.h"

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

/**
 * @brief A BT::ConditionNode that returns SUCCESS when the IsPathValid
 * service returns true and FAILURE otherwise
 */
class IsPathValidCondition : public BT::ConditionNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::IsPathValidCondition
     * @param condition_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    IsPathValidCondition(const std::string& condition_name,
                         const BT::NodeConfiguration& conf);

    IsPathValidCondition() = delete;

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Function to read parameters and initialize class variables
     */
    void initialize();

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts() {
        // Register JSON definitions for the types used in the ports
        BT::RegisterJsonDefinition<commsgs::planning_msgs::Path>();
        BT::RegisterJsonDefinition<std::chrono::milliseconds>();

        return {BT::InputPort<commsgs::planning_msgs::Path>("path",
                                                            "Path to Check"),
                BT::InputPort<std::chrono::milliseconds>("server_timeout"),
                BT::InputPort<unsigned int>("max_cost", 253,
                                            "Maximum cost of the path"),
                BT::InputPort<bool>(
                    "consider_unknown_as_obstacle", false,
                    "Whether to consider unknown cost as obstacle")};
    }

private:
    std::shared_ptr<::autolink::Node> node_;
    std::shared_ptr<::autolink::Client<proto::IsPathValid::Request,
                                       proto::IsPathValid::Response>>
        client_;
    // The timeout value while waiting for a response from the
    // is path valid service
    std::chrono::milliseconds server_timeout_;
    unsigned int max_cost_;
    bool consider_unknown_as_obstacle_;
};

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy