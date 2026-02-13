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

#include "autonomy/tasks/behavior_tree/plugins/condition/is_stuck_condition.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

IsStuckCondition::IsStuckCondition(const std::string& condition_name, const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf),
      is_stuck_(false),
      odom_history_size_(10),
      current_accel_(0.0),
      brake_accel_limit_(-10.0) {
    node_ = config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
    odom_sub_ = node_->CreateReader<commsgs::planning_msgs::Odometry>(
        "odom", std::bind(&IsStuckCondition::onOdomReceived, this, std::placeholders::_1));

    AINFO << "Initialized an IsStuckCondition BT node";
}

IsStuckCondition::~IsStuckCondition() {
    AINFO << "Shutting down IsStuckCondition BT node";
}

void IsStuckCondition::onOdomReceived(const std::shared_ptr<commsgs::planning_msgs::Odometry>& msg) {
    while (odom_history_.size() >= odom_history_size_) {
        odom_history_.pop_front();
    }
    odom_history_.push_back(*msg);
    updateStates();
}

BT::NodeStatus IsStuckCondition::tick() {
    if (is_stuck_) {
        logStuck("Robot got stuck!");
        return BT::NodeStatus::SUCCESS;  // Successfully detected a stuck
                                         // condition
    }

    logStuck("Robot is free");
    return BT::NodeStatus::FAILURE;  // Failed to detected a stuck condition
}

void IsStuckCondition::logStuck(const std::string& msg) const {
    static std::string prev_msg;

    if (msg == prev_msg) {
        return;
    }

    AINFO << msg;
    prev_msg = msg;
}

void IsStuckCondition::updateStates() {
    // Approximate acceleration
    if (odom_history_.size() > 2) {
        auto curr_odom = odom_history_.end()[-1];
        double curr_time = static_cast<double>(curr_odom.header.stamp.sec);
        curr_time += (static_cast<double>(curr_odom.header.stamp.nanosec)) * 1e-9;

        auto prev_odom = odom_history_.end()[-2];
        double prev_time = static_cast<double>(prev_odom.header.stamp.sec);
        prev_time += (static_cast<double>(prev_odom.header.stamp.nanosec)) * 1e-9;

        double dt = curr_time - prev_time;
        double vel_diff = static_cast<double>(curr_odom.twist.twist.linear.x - prev_odom.twist.twist.linear.x);
        current_accel_ = vel_diff / dt;
    }

    is_stuck_ = isStuck();
}

bool IsStuckCondition::isStuck() {
    if (current_accel_ < brake_accel_limit_) {
        AINFO << "Current deceleration is beyond brake limit."
              << " brake limit: " << brake_accel_limit_ << ", current accel: " << current_accel_;
        return true;
    }

    return false;
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::condition::IsStuckCondition>("IsStuck");
}
