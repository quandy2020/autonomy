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

#include "autonomy/tasks/behavior_tree/plugins/condition/is_path_valid_condition.hpp"

#include <future>

#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/utils/path_validation_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

IsPathValidCondition::IsPathValidCondition(const std::string& condition_name,
                                           const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf),
      server_timeout_(std::chrono::milliseconds(20)),
      max_cost_(253),
      consider_unknown_as_obstacle_(false) {}

void IsPathValidCondition::initialize() {
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
    getInput<unsigned int>("max_cost", max_cost_);
    getInput<bool>("consider_unknown_as_obstacle",
                   consider_unknown_as_obstacle_);
}

BT::NodeStatus IsPathValidCondition::tick() {
    if (!BT::isStatusActive(status())) {
        initialize();
    }

    commsgs::planning_msgs::Path path;
    if (!getInput("path", path) || path.poses.empty()) {
        return BT::NodeStatus::FAILURE;
    }

    auto ctx = config().blackboard->get<std::shared_ptr<common::TaskContext>>(
        "task_context");
    auto costmap =
        ctx && ctx->global_costmap ? ctx->global_costmap : nullptr;
    if (!costmap) {
        return BT::NodeStatus::FAILURE;
    }

    return utils::IsPathValidOnCostmap(costmap, path, max_cost_,
                                       consider_unknown_as_obstacle_)
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::FAILURE;
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::
                                 condition::IsPathValidCondition>(
        "IsPathValid");
}
