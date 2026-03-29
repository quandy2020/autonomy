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

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_action_node.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps
 * nav2_msgs::action::ComputeRoute
 */
class ComputeRouteAction : public BtActionNode<proto::ComputeRouteAction>
{
    using Action = proto::ComputeRouteAction;
    using ActionResult = Action::Result;

public:
    /**
     * @brief A constructor for nav2_behavior_tree::ComputeRoute
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    ComputeRouteAction(const std::string& xml_tag_name,
                       const std::string& action_name,
                       const BT::NodeConfiguration& conf);

    /**
     * @brief Function to perform some user-defined operation on tick
     */
    void on_tick() override;

    /**
     * @brief Function to perform some user-defined operation upon successful
     * completion of the action
     */
    BT::NodeStatus on_success() override;

    /**
     * @brief Function to perform some user-defined operation upon abortion of
     * the action
     */
    BT::NodeStatus on_aborted() override;

    /**
     * @brief Function to perform some user-defined operation upon cancellation
     * of the action
     */
    BT::NodeStatus on_cancelled() override;

    /**
     * @brief Function to perform work in a BT Node when the action server times
     * out Such as setting the error code ID status to timed out for action
     * clients.
     */
    void on_timeout() override;

    /**
     * \brief Override required by the a BT action. Cancel the action and set
     * the path output
     */
    void halt() override;

    /**
     * @brief Reset output port values on failure
     */
    void resetPorts();

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<unsigned int>("start_id", "ID of the start node"),
            BT::InputPort<unsigned int>("goal_id", "ID of the goal node"),
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>(
                "start",
                "Start pose of the path if overriding current robot pose and "
                "using poses over IDs"),
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>(
                "goal", "Goal pose of the path if using poses over IDs"),
            BT::InputPort<bool>(
                "use_start", false,
                "Whether to use the start pose or the robot's current pose"),
            BT::InputPort<bool>(
                "use_poses", false,
                "Whether to use poses or IDs for start and goal"),
            BT::OutputPort<proto::Route>(
                "route", "The route computed by ComputeRoute node"),
            BT::OutputPort<commsgs::builtin_interfaces::Duration>(
                "planning_time", "Time taken to compute route"),
            BT::OutputPort<commsgs::planning_msgs::Path>(
                "path", "Path created by ComputeRoute node"),
            BT::OutputPort<int32_t>("error_code_id",
                                    "The compute route error code"),
            BT::OutputPort<std::string>("error_msg",
                                        "The compute route error msg"),
        });
    }
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
