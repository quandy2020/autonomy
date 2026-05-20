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

#include "autonomy/tasks/behavior_tree/plugins/action/wait_action.hpp"

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

WaitAction::WaitAction(const std::string& xml_tag_name,
                       const BT::NodeConfiguration& conf)
    : BtStatefulActionNode(xml_tag_name, conf),
      duration_sec_(0.0) {}

BT::NodeStatus WaitAction::onStart() {
    duration_sec_ = 1.0;
    if (!getInput("wait_duration", duration_sec_)) {
        AWARN << "wait_duration port missing, using 0s.";
        duration_sec_ = 0.0;
    }
    if (duration_sec_ <= 0.0) {
        return BT::NodeStatus::SUCCESS;
    }
    start_ = commsgs::builtin_interfaces::Time::Now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitAction::onRunning() {
    const auto now = commsgs::builtin_interfaces::Time::Now();
    const int64_t start_ns =
        static_cast<int64_t>(start_.sec) * 1000000000LL + start_.nanosec;
    const int64_t now_ns =
        static_cast<int64_t>(now.sec) * 1000000000LL + now.nanosec;
    const double elapsed =
        commsgs::builtin_interfaces::Duration::FromNanoseconds(now_ns - start_ns)
            .Seconds();
    if (elapsed >= duration_sec_) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::plugins::action::WaitAction>("Wait");
}
