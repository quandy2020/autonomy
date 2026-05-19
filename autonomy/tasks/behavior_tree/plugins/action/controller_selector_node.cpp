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

#include "autonomy/tasks/behavior_tree/plugins/action/controller_selector_node.hpp"

#include <functional>
#include <set>
#include <string>
#include <vector>

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

ControllerSelector::ControllerSelector(const std::string& name,
                                       const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(name, conf) {
    initialize();
}

void ControllerSelector::initialize() {
    createROSInterfaces();
}

void ControllerSelector::createROSInterfaces() {
    std::string topic_new;
    getInput("topic_name", topic_new);
    if (topic_new != topic_name_) {
        topic_name_ = topic_new;
    }
}

BT::NodeStatus ControllerSelector::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
    }

    // This behavior always use the last selected controller received from the
    // topic input. When no input is specified it uses the default controller.
    // If the default controller is not specified then we work in "required
    // controller mode": In this mode, the behavior returns failure if the
    // controller selection is not received from the topic input.
    if (last_selected_controller_.empty()) {
        std::string default_controller;
        getInput("default_controller", default_controller);
        if (default_controller.empty()) {
            return BT::NodeStatus::FAILURE;
        } else {
            last_selected_controller_ = default_controller;
        }
    }

    setOutput("selected_controller", last_selected_controller_);

    return BT::NodeStatus::SUCCESS;
}

void ControllerSelector::callbackControllerSelect(
    std::shared_ptr<const commsgs::std_msgs::String> msg) {
    last_selected_controller_ = msg->data;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::plugins::action::ControllerSelector>(
        "ControllerSelector");
}
