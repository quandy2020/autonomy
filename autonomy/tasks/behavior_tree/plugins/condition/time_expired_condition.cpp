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

#include "autonomy/tasks/behavior_tree/plugins/condition/time_expired_condition.hpp"

#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

TimeExpiredCondition::TimeExpiredCondition(const std::string& condition_name, const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf), period_(1.0) {}

void TimeExpiredCondition::initialize() {
    getInput("seconds", period_);
    node_ = config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
    start_ = autonomy::commsgs::builtin_interfaces::Time::Now();
}

BT::NodeStatus TimeExpiredCondition::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
    }

    if (!BT::isStatusActive(status())) {
        start_ = autonomy::commsgs::builtin_interfaces::Time::Now();
        return BT::NodeStatus::FAILURE;
    }

    // Determine how long it's been since we've started this iteration
    auto current_time = autonomy::commsgs::builtin_interfaces::Time::Now();
    int64_t current_ns = static_cast<int64_t>(current_time.sec) * 1000000000LL + current_time.nanosec;
    int64_t start_ns = static_cast<int64_t>(start_.sec) * 1000000000LL + start_.nanosec;
    int64_t elapsed_ns = current_ns - start_ns;
    auto elapsed = commsgs::builtin_interfaces::Duration::FromNanoseconds(elapsed_ns);

    // Now, get that in seconds
    double seconds = elapsed.Seconds();

    if (seconds < period_) {
        return BT::NodeStatus::FAILURE;
    }

    start_ = commsgs::builtin_interfaces::Time::Now();  // Reset the timer
    return BT::NodeStatus::SUCCESS;
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::condition::TimeExpiredCondition>("TimeExpired");
}
