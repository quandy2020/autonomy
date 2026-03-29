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

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_action_node.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief A autonomy::tasks::behavior_tree::BtActionNode class that wraps
 * proto::SmoothPathAction
 */
class SmoothPathAction : public BtActionNode<proto::SmoothPathAction>
{
    using Action = proto::SmoothPathAction;
    using ActionResult = Action::Result;

public:
    /**
     * @brief A constructor for
     * autonomy::tasks::behavior_tree::plugins::action::SmoothPathAction
     * @param xml_tag_name Name for the XML tag for this node
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    SmoothPathAction(const std::string& xml_tag_name,
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
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedPorts() {
        // Register JSON definitions for the types used in the ports
        BT::RegisterJsonDefinition<commsgs::planning_msgs::Path>();

        return providedBasicPorts({
            BT::InputPort<commsgs::planning_msgs::Path>("unsmoothed_path",
                                                        "Path to be smoothed"),
            BT::InputPort<double>("max_smoothing_duration", 3.0,
                                  "Maximum smoothing duration"),
            BT::InputPort<bool>(
                "check_for_collisions", false,
                "If true collision check will be performed after smoothing"),
            BT::InputPort<std::string>("smoother_id", ""),
            BT::OutputPort<commsgs::planning_msgs::Path>(
                "smoothed_path", "Path smoothed by SmootherServer node"),
            BT::OutputPort<double>("smoothing_duration",
                                   "Time taken to smooth path"),
            BT::OutputPort<bool>(
                "was_completed",
                "True if smoothing was not interrupted by time limit"),
            BT::OutputPort<int32_t>("error_code_id",
                                    "The smooth path error code"),
            BT::OutputPort<std::string>("error_msg",
                                        "The smooth path error msg"),
        });
    }
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
