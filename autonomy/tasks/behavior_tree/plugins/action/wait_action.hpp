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

#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/tasks/behavior_tree/bt_stateful_action_node.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

class WaitAction : public BtStatefulActionNode
{
public:
    WaitAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("wait_duration", 1.0, "Wait time (s)")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;

private:
    commsgs::builtin_interfaces::Time start_;
    double duration_sec_;
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
