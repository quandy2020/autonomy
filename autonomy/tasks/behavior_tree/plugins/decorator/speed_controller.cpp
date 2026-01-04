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

#include "autonomy/tasks/behavior_tree/plugins/decorator/speed_controller.hpp"

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace decorator {

SpeedController::SpeedController(const std::string& name,
                                 const BT::NodeConfiguration& conf)
    : BT::DecoratorNode(name, conf),
      first_tick_(false),
      period_(1.0),
      min_rate_(0.1),
      max_rate_(1.0),
      min_speed_(0.0),
      max_speed_(0.5) {
    // node_ =
    node_ = config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
    // TODO: Get node from blackboard

    getInput("min_rate", min_rate_);
    getInput("max_rate", max_rate_);
    getInput("min_speed", min_speed_);
    getInput("max_speed", max_speed_);

    if (min_rate_ <= 0.0 || max_rate_ <= 0.0) {
        std::string err_msg = "SpeedController node cannot have rate <= 0.0";
        // LOG(FATAL) << err_msg; // TODO: Add logging
        throw BT::BehaviorTreeException(err_msg);
    }

    d_rate_ = max_rate_ - min_rate_;
    d_speed_ = max_speed_ - min_speed_;

    odom_smoother_ =
        config().blackboard->get<std::shared_ptr<control::utils::OdomSmoother>>(
            "odom_smoother");
    // TODO: Implement odom_smoother for autonomy framework
}

inline BT::NodeStatus SpeedController::tick() {
    if (!BT::isStatusActive(status())) {
        // Reset since we're starting a new iteration of
        // the speed controller (moving from IDLE to RUNNING)
        // BT::getInputOrBlackboard("goals", goals_); // Goals type not yet
        // implemented BT::getInputOrBlackboard("goal", goal_);
        period_ = 1.0 / max_rate_;
        start_ = commsgs::builtin_interfaces::Time::Now();
        first_tick_ = true;
    }

    commsgs::geometry_msgs::PoseStamped current_goal;
    GetInputOrBlackboard("goal", current_goal);

    // Compare goals by checking position and orientation
    bool goal_changed =
        (goal_.pose.position.x != current_goal.pose.position.x ||
         goal_.pose.position.y != current_goal.pose.position.y ||
         goal_.pose.position.z != current_goal.pose.position.z ||
         goal_.pose.orientation.x != current_goal.pose.orientation.x ||
         goal_.pose.orientation.y != current_goal.pose.orientation.y ||
         goal_.pose.orientation.z != current_goal.pose.orientation.z ||
         goal_.pose.orientation.w != current_goal.pose.orientation.w);

    if (goal_changed) {
        // Reset state and set period to max since we have a new goal
        period_ = 1.0 / max_rate_;
        start_ = commsgs::builtin_interfaces::Time::Now();
        first_tick_ = true;
        goal_ = current_goal;
    }

    setStatus(BT::NodeStatus::RUNNING);

    // Calculate elapsed time manually
    auto current_time = commsgs::builtin_interfaces::Time::Now();
    int64_t current_ns = static_cast<int64_t>(current_time.sec) * 1000000000LL +
                         current_time.nanosec;
    int64_t start_ns =
        static_cast<int64_t>(start_.sec) * 1000000000LL + start_.nanosec;
    int64_t elapsed_ns = current_ns - start_ns;
    auto elapsed =
        commsgs::builtin_interfaces::Duration::FromNanoseconds(elapsed_ns);

    // The child gets ticked the first time through and any time the period has
    // expired. In addition, once the child begins to run, it is ticked each
    // time 'til completion
    if (first_tick_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
        elapsed.Seconds() >= period_) {
        first_tick_ = false;

        // update period if the last period is exceeded
        if (elapsed.Seconds() >= period_) {
            updatePeriod();
            start_ = commsgs::builtin_interfaces::Time::Now();
        }

        return child_node_->executeTick();
    }

    return status();
}

}  // namespace decorator
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::plugins::decorator::SpeedController>(
        "SpeedController");
}
