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

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_action_node.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps
 * nav2_msgs::action::ComputePathToPose
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class ComputePathToPoseAction
    : public BtActionNode<
          autonomy::tasks::behavior_tree::proto::ComputePathToPoseAction>
{
    using Action =
        autonomy::tasks::behavior_tree::proto::ComputePathToPoseAction;
    using ActionResult = Action::Result;

public:
    /**
     * @brief A constructor for
     * autonomy::tasks::behavior_tree::plugins::action::ComputePathToPoseAction
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    ComputePathToPoseAction(const std::string& xml_tag_name,
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
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedPorts() {
        // Register JSON definitions for the types used in the ports
        BT::RegisterJsonDefinition<commsgs::planning_msgs::Path>();
        BT::RegisterJsonDefinition<commsgs::geometry_msgs::PoseStamped>();

        return providedBasicPorts({
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>(
                "goal", "Destination to plan to"),
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>(
                "start",
                "Used as the planner start pose instead of the current robot "
                "pose, "
                "if use_start is"
                " not false (i.e. not provided or set to true)"),
            BT::InputPort<bool>("use_start",
                                "For using or not using (i.e. ignoring) the "
                                "provided start pose"),
            BT::InputPort<std::string>(
                "planner_id", "",
                "Mapped name to the planner plugin type to use"),
            BT::OutputPort<commsgs::planning_msgs::Path>(
                "path", "Path created by ComputePathToPose node"),
            BT::OutputPort<int32_t>("error_code_id",
                                    "The compute path to pose error code"),
            BT::OutputPort<std::string>("error_msg",
                                        "The compute path to pose error msg"),
        });
    }
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy