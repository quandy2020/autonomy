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

#include "autonomy/tasks/behavior_tree/plugins/action/smoother_selector_node.hpp"

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

SmootherSelector::SmootherSelector(const std::string& xml_tag_name,
                                   const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(xml_tag_name, conf) {}

void SmootherSelector::readDefaultFromPorts() {
    std::string default_smoother;
    getInput("default_smoother", default_smoother);
    if (default_smoother.empty()) {
        default_smoother =
            common::DefaultSmootherFromBlackboard(config().blackboard);
    }
    if (!default_smoother.empty()) {
        last_selected_smoother_ = default_smoother;
    }
}

BT::NodeStatus SmootherSelector::tick() {
    if (last_selected_smoother_.empty()) {
        readDefaultFromPorts();
    }
    if (last_selected_smoother_.empty()) {
        return BT::NodeStatus::FAILURE;
    }

    setOutput("selected_smoother", last_selected_smoother_);
    config().blackboard->set("selected_smoother", last_selected_smoother_);  // NOLINT

    return BT::NodeStatus::SUCCESS;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::plugins::action::SmootherSelector>(
        "SmootherSelector");
}
