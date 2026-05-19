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

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/json_export.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace decorator {

/**
 * @brief A BT::DecoratorNode that subscribes to a goal topic and updates
 * the current goal on the blackboard
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class GoalUpdater : public BT::DecoratorNode
{
public:
    /**
     * @brief A constructor for GoalUpdater
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    GoalUpdater(const std::string& xml_tag_name,
                const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts() {
        // Register JSON definitions for the types used in the ports
        BT::RegisterJsonDefinition<commsgs::geometry_msgs::PoseStamped>();
        BT::RegisterJsonDefinition<commsgs::planning_msgs::Goals>();

        return {BT::InputPort<commsgs::geometry_msgs::PoseStamped>(
                    "input_goal", "Original Goal"),
                BT::InputPort<commsgs::planning_msgs::Goals>("input_goals",
                                                             "Original Goals"),
                BT::OutputPort<commsgs::geometry_msgs::PoseStamped>(
                    "output_goal", "Received Goal by subscription"),
                BT::OutputPort<commsgs::planning_msgs::Goals>(
                    "output_goals", "Received Goals by subscription")};
    }

private:
    /**
     * @brief Function to read parameters and initialize class variables
     */
    void initialize();
    /**
     * @brief Function to create ROS interfaces
     */
    void createROSInterfaces();
    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Callback function for goal update topic
     * @param msg Shared pointer to geometry_msgs::msg::PoseStamped message
     */
    void callback_updated_goal(
        const commsgs::geometry_msgs::PoseStamped::SharedPtr msg);

    /**
     * @brief Callback function for goals update topic
     * @param msg Shared pointer to commsgs::planning_msgs::Goals message
     */
    void callback_updated_goals(
        const commsgs::planning_msgs::Goals::SharedPtr msg);

    commsgs::geometry_msgs::PoseStamped last_goal_received_;
    bool last_goal_received_set_{false};
    commsgs::planning_msgs::Goals last_goals_received_;
    bool last_goals_received_set_{false};
    std::string goal_updater_topic_;
    std::string goals_updater_topic_;
};

}  // namespace decorator
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy