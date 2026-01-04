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

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/json_export.h"

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/utils/odometry_utils.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace decorator {

/**
 * @brief A BT::DecoratorNode that ticks its child every at a rate proportional
 * to the speed of the robot. If the robot travels faster, this node will tick
 * its child at a higher frequency and reduce the tick frequency if the robot
 * slows down
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class SpeedController : public BT::DecoratorNode
{
public:
    /**
     * @brief A constructor for
     * autonomy::tasks::behavior_tree::plugins::decorator::SpeedController
     * @param name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    SpeedController(const std::string& name, const BT::NodeConfiguration& conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts() {
        // Register JSON definitions for the types used in the ports
        // BT::RegisterJsonDefinition<commsgs::geometry_msgs::PoseStamped>(); //
        // TODO: Implement JSON conversion
        //  BT::RegisterJsonDefinition<commsgs::planning_msgs::Goals>();

        return {
            BT::InputPort<double>("min_rate", 0.1, "Minimum rate"),
            BT::InputPort<double>("max_rate", 1.0, "Maximum rate"),
            BT::InputPort<double>("min_speed", 0.0, "Minimum speed"),
            BT::InputPort<double>("max_speed", 0.5, "Maximum speed"),
            // BT::InputPort<commsgs::planning_msgs::Goals>("goals", "Vector of
            // navigation goals"), // Goals type not yet implemented
            BT::InputPort<commsgs::geometry_msgs::PoseStamped>(
                "goal", "Navigation goal"),
        };
    }

private:
    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Scale the rate based speed
     * @return double Rate scaled by speed limits and clamped
     */
    inline double getScaledRate(const double& speed) {
        return std::max(
            std::min((((speed - min_speed_) / d_speed_) * d_rate_) + min_rate_,
                     max_rate_),
            min_rate_);
    }

    /**
     * @brief Update period based on current smoothed speed and reset timer
     */
    inline void updatePeriod() {
        // auto velocity = odom_smoother_->getTwist(); // OdomSmoother not yet
        // implemented
        // TODO: Get velocity from odometry
        // commsgs::geometry_msgs::Twist velocity; // Placeholder
        // TODO: Get velocity from odometry and calculate speed
        double speed = 0.0;  // Placeholder
        double rate = getScaledRate(speed);
        period_ = 1.0 / rate;
    }

    std::shared_ptr<::autolink::Node> node_;

    // To keep track of time to reset
    commsgs::builtin_interfaces::Time start_;

    // To get a smoothed velocity
    std::shared_ptr<control::utils::OdomSmoother> odom_smoother_;  // Not yet
    // implemented

    bool first_tick_;

    // Time period after which child node should be ticked
    double period_;

    // Rates thresholds to tick child node
    double min_rate_;
    double max_rate_;
    double d_rate_;

    // Speed thresholds
    double min_speed_;
    double max_speed_;
    double d_speed_;

    // current goal
    commsgs::geometry_msgs::PoseStamped goal_;
    // commsgs::planning_msgs::Goals goals_;
};

}  // namespace decorator
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy