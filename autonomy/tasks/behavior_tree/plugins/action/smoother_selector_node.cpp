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

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

SmootherSelector::SmootherSelector(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(name, conf) {
    initialize();
}

void SmootherSelector::initialize() {
    createROSInterfaces();
}

void SmootherSelector::createROSInterfaces() {
    std::string topic_new;
    getInput("topic_name", topic_new);
    if (topic_new != topic_name_ || !smoother_selector_sub_) {
        topic_name_ = topic_new;
        node_ = config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");

        smoother_selector_sub_ = node_->CreateReader<commsgs::std_msgs::String>(
            topic_name_, [this](std::shared_ptr<const commsgs::std_msgs::String> msg) { callbackSmootherSelect(msg); });
    }
}

void SmootherSelector::callbackSmootherSelect(std::shared_ptr<const commsgs::std_msgs::String> msg) {
    if (msg && !msg->data.empty()) {
        last_selected_smoother_ = msg->data;
    }
}

BT::NodeStatus SmootherSelector::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
    }

    // Process callbacks if needed (autolink handles this internally via
    // readers)

    // This behavior always use the last selected smoother received from the
    // topic input. When no input is specified it uses the default smoother. If
    // the default smoother is not specified then we work in "required smoother
    // mode": In this mode, the behavior returns failure if the smoother
    // selection is not received from the topic input.
    if (last_selected_smoother_.empty()) {
        std::string default_smoother;
        getInput("default_smoother", default_smoother);
        if (default_smoother.empty()) {
            return BT::NodeStatus::FAILURE;
        } else {
            last_selected_smoother_ = default_smoother;
        }
    }

    setOutput("selected_smoother", last_selected_smoother_);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::action::SmootherSelector>("SmootherSelector");
}
