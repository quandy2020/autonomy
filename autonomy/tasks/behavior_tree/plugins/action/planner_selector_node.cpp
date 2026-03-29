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

#include "autonomy/tasks/behavior_tree/plugins/action/planner_selector_node.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

PlannerSelector::PlannerSelector(const std::string& name,
                                 const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(name, conf) {
    initialize();
}

void PlannerSelector::initialize() {
    createROSInterfaces();
}

void PlannerSelector::createROSInterfaces() {
    std::string topic_new;
    getInput("topic_name", topic_new);
    if (topic_new != topic_name_ || !planner_selector_sub_) {
        topic_name_ = topic_new;
        node_ =
            config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
        planner_selector_sub_ = node_->CreateReader<commsgs::std_msgs::String>(
            topic_name_,
            [this](std::shared_ptr<const commsgs::std_msgs::String> msg) {
                callbackPlannerSelect(msg);
            });
    }
}

BT::NodeStatus PlannerSelector::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
    }

    // This behavior always use the last selected planner received from the
    // topic input. When no input is specified it uses the default planner. If
    // the default planner is not specified then we work in "required planner
    // mode": In this mode, the behavior returns failure if the planner
    // selection is not received from the topic input.
    if (last_selected_planner_.empty()) {
        std::string default_planner;
        getInput("default_planner", default_planner);
        if (default_planner.empty()) {
            return BT::NodeStatus::FAILURE;
        } else {
            last_selected_planner_ = default_planner;
        }
    }

    setOutput("selected_planner", last_selected_planner_);

    return BT::NodeStatus::SUCCESS;
}

void PlannerSelector::callbackPlannerSelect(
    std::shared_ptr<const commsgs::std_msgs::String> msg) {
    last_selected_planner_ = msg->data;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<
        autonomy::tasks::behavior_tree::plugins::action::PlannerSelector>(
        "PlannerSelector");
}
