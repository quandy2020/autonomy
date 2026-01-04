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

#include "autonomy/tasks/behavior_tree/plugins/condition/goal_updated_condition.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

GoalUpdatedCondition::GoalUpdatedCondition(const std::string& condition_name,
                                           const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf) {}

BT::NodeStatus GoalUpdatedCondition::tick() {
    if (!BT::isStatusActive(status())) {
        GetInputOrBlackboard("goals", goals_);
        GetInputOrBlackboard("goal", goal_);
        return BT::NodeStatus::FAILURE;
    }

    commsgs::planning_msgs::Goals current_goals;
    commsgs::geometry_msgs::PoseStamped current_goal;
    GetInputOrBlackboard("goals", current_goals);
    GetInputOrBlackboard("goal", current_goal);

    // Manual comparison for PoseStamped
    bool goal_changed =
        (goal_.pose.position.x != current_goal.pose.position.x ||
         goal_.pose.position.y != current_goal.pose.position.y ||
         goal_.pose.position.z != current_goal.pose.position.z ||
         goal_.pose.orientation.x != current_goal.pose.orientation.x ||
         goal_.pose.orientation.y != current_goal.pose.orientation.y ||
         goal_.pose.orientation.z != current_goal.pose.orientation.z ||
         goal_.pose.orientation.w != current_goal.pose.orientation.w);

    // Manual comparison for Goals
    bool goals_changed = (goals_.goals.size() != current_goals.goals.size());
    if (!goals_changed) {
        for (size_t i = 0; i < goals_.goals.size(); ++i) {
            const auto& g1 = goals_.goals[i];
            const auto& g2 = current_goals.goals[i];
            if (g1.pose.position.x != g2.pose.position.x ||
                g1.pose.position.y != g2.pose.position.y ||
                g1.pose.position.z != g2.pose.position.z ||
                g1.pose.orientation.x != g2.pose.orientation.x ||
                g1.pose.orientation.y != g2.pose.orientation.y ||
                g1.pose.orientation.z != g2.pose.orientation.z ||
                g1.pose.orientation.w != g2.pose.orientation.w) {
                goals_changed = true;
                break;
            }
        }
    }

    if (goal_changed || goals_changed) {
        goal_ = current_goal;
        goals_ = current_goals;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::
                                 condition::GoalUpdatedCondition>(
        "GoalUpdated");
}
