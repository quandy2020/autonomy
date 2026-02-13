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

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/json_export.h"

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"
#include "autonomy/tasks/navigator/proto/msg.pb.h"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

/**
 * @brief A BT::ConditionNode that returns SUCCESS when a specified goal
 * is reached and FAILURE otherwise
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class GoalReachedCondition : public BT::ConditionNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::GoalReachedCondition
     * @param condition_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    GoalReachedCondition(const std::string& condition_name, const BT::NodeConfiguration& conf);

    GoalReachedCondition() = delete;

    /**
     * @brief A destructor for nav2_behavior_tree::GoalReachedCondition
     */
    ~GoalReachedCondition() override;

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
     * @brief Checks if the current robot pose lies within a given distance from
     * the goal
     * @return bool true when goal is reached, false otherwise
     */
    bool isGoalReached();

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts() {
        // Register JSON definitions for the types used in the ports
        BT::RegisterJsonDefinition<commsgs::geometry_msgs::PoseStamped>();

        return {BT::InputPort<commsgs::geometry_msgs::PoseStamped>("goal", "Destination"),
                BT::InputPort<std::string>("robot_base_frame", "Robot base frame"),
                BT::InputPort<double>("goal_reached_tol", 0.25, "Tolerance for considering goal reached"),
                BT::InputPort<double>("transform_tolerance", 0.1, "Transform tolerance")};
    }

protected:
    /**
     * @brief Cleanup function
     */
    void cleanup() {}

private:
    std::shared_ptr<::autolink::Node> node_;
    std::shared_ptr<autonomy::transform::Buffer> tf_;

    double goal_reached_tol_;
    double transform_tolerance_;
    std::string robot_base_frame_;
};

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy