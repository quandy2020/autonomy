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

#include "autonomy/tasks/behavior_tree/plugins/decorator/goal_updated_controller.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace decorator {

GoalUpdatedController::GoalUpdatedController(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::DecoratorNode(name, conf) {}

BT::NodeStatus GoalUpdatedController::tick() {
  if (!BT::isStatusActive(status())) {
    // Reset since we're starting a new iteration of
    // the goal updated controller (moving from IDLE to RUNNING)

    GetInputOrBlackboard("goals", goals_);
    GetInputOrBlackboard("goal", goal_);

    goal_was_updated_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  commsgs::planning_msgs::Goals current_goals;
  GetInputOrBlackboard("goals", current_goals);
  commsgs::geometry_msgs::PoseStamped current_goal;
  GetInputOrBlackboard("goal", current_goal);

  // Compare goals by checking if they are different
  bool goal_changed = false;
  bool goals_changed = false;

  // Simple comparison - check if sizes or first elements differ
  if (goal_.header.frame_id != current_goal.header.frame_id || goal_.pose.position.x != current_goal.pose.position.x ||
      goal_.pose.position.y != current_goal.pose.position.y) {
    goal_changed = true;
  }

  if (goals_.goals.size() != current_goals.goals.size() ||
      (goals_.goals.size() > 0 && current_goals.goals.size() > 0 &&
       (goals_.goals[0].pose.position.x != current_goals.goals[0].pose.position.x ||
        goals_.goals[0].pose.position.y != current_goals.goals[0].pose.position.y))) {
    goals_changed = true;
  }

  if (goal_changed || goals_changed) {
    goal_ = current_goal;
    goals_ = current_goals;
    goal_was_updated_ = true;
  }

  // The child gets ticked the first time through and any time the goal has
  // changed or preempted. In addition, once the child begins to run, it is
  // ticked each time 'til completion
  if ((child_node_->status() == BT::NodeStatus::RUNNING) || goal_was_updated_) {
    goal_was_updated_ = false;
    return child_node_->executeTick();
  }

  return status();
}

}  // namespace decorator
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::decorator::GoalUpdatedController>(
      "GoalUpdatedController");
}
