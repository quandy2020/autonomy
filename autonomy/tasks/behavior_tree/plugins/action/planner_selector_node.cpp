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

#include "autonomy/tasks/behavior_tree/bt_utils.hpp"
#include "autonomy/tasks/utils/planner_id_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

PlannerSelector::PlannerSelector(const std::string& xml_tag_name,
                                 const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus PlannerSelector::tick() {
    if (last_selected_planner_.empty()) {
        std::string default_planner;
        getInput("default_planner", default_planner);
        if (default_planner.empty()) {
            default_planner =
                common::DefaultPlannerFromBlackboard(config().blackboard);
        }
        if (default_planner.empty()) {
            return BT::NodeStatus::FAILURE;
        }
        last_selected_planner_ = default_planner;
    }

    const std::string default_id =
        common::DefaultPlannerFromBlackboard(config().blackboard);
    last_selected_planner_ =
        utils::ResolvePlannerId(last_selected_planner_, default_id);
    setOutput("selected_planner", last_selected_planner_);
    config().blackboard->set("selected_planner", last_selected_planner_);  // NOLINT

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
        autonomy::tasks::behavior_tree::plugins::action::PlannerSelector>(
        "PlannerSelector");
}
