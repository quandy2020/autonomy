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

#include "autonomy/tasks/behavior_tree/plugins/condition/is_stopped_condition.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

IsStoppedCondition::IsStoppedCondition(const std::string& condition_name, const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf),
      velocity_threshold_(0.01),
      duration_stopped_(1000ms),
      stopped_stamp_(commsgs::builtin_interfaces::Time(0, 0)) {
    node_ = config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
    odom_smoother_ = config().blackboard->get<std::shared_ptr<control::utils::OdomSmoother>>("odom_smoother");
}

IsStoppedCondition::~IsStoppedCondition() {
    AINFO << "Shutting down IsStoppedCondition BT node";
}

BT::NodeStatus IsStoppedCondition::tick() {
    getInput("velocity_threshold", velocity_threshold_);
    getInput("duration_stopped", duration_stopped_);

    auto twist = odom_smoother_->getRawTwistStamped();

    // if there is no timestamp, set it to now
    if (twist.header.stamp.sec == 0 && twist.header.stamp.nanosec == 0) {
        twist.header.stamp = commsgs::builtin_interfaces::Time::Now();
    }

    commsgs::builtin_interfaces::Time zero_time(0, 0);

    if (abs(twist.twist.linear.x) < velocity_threshold_ && abs(twist.twist.linear.y) < velocity_threshold_ &&
        abs(twist.twist.angular.z) < velocity_threshold_) {
        if (stopped_stamp_ == zero_time) {
            stopped_stamp_ = twist.header.stamp;
        }

        auto current_time = commsgs::builtin_interfaces::Time::Now();
        auto elapsed_ns = current_time.Nanoseconds() - stopped_stamp_.Nanoseconds();
        auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(duration_stopped_).count();

        if (elapsed_ns > static_cast<uint32>(duration_ns)) {
            stopped_stamp_ = zero_time;
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::RUNNING;
        }

    } else {
        stopped_stamp_ = zero_time;
        return BT::NodeStatus::FAILURE;
    }
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::condition::IsStoppedCondition>("IsStopped");
}
