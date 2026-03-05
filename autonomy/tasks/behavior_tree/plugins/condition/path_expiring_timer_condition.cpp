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

#include "autonomy/tasks/behavior_tree/plugins/condition/path_expiring_timer_condition.hpp"

#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

PathExpiringTimerCondition::PathExpiringTimerCondition(const std::string& condition_name,
                                                       const BT::NodeConfiguration& conf)
    : BT::ConditionNode(condition_name, conf), period_(1.0), first_time_(true) {
  node_ = config().blackboard->get<std::shared_ptr<::autolink::Node>>("node");
}

BT::NodeStatus PathExpiringTimerCondition::tick() {
  if (first_time_) {
    getInput("seconds", period_);
    getInput("path", prev_path_);
    first_time_ = false;
    start_ = autonomy::commsgs::builtin_interfaces::Time::Now();
    return BT::NodeStatus::FAILURE;
  }

  // Grab the new path
  commsgs::planning_msgs::Path path;
  getInput("path", path);

  // Reset timer if the path has been updated
  // Manual comparison for Path
  bool path_changed = (prev_path_.poses.size() != path.poses.size());
  if (!path_changed) {
    for (size_t i = 0; i < prev_path_.poses.size(); ++i) {
      const auto& p1 = prev_path_.poses[i];
      const auto& p2 = path.poses[i];
      if (p1.pose.position.x != p2.pose.position.x || p1.pose.position.y != p2.pose.position.y ||
          p1.pose.position.z != p2.pose.position.z || p1.pose.orientation.x != p2.pose.orientation.x ||
          p1.pose.orientation.y != p2.pose.orientation.y || p1.pose.orientation.z != p2.pose.orientation.z ||
          p1.pose.orientation.w != p2.pose.orientation.w) {
        path_changed = true;
        break;
      }
    }
  }
  if (path_changed) {
    prev_path_ = path;
    start_ = autonomy::commsgs::builtin_interfaces::Time::Now();
  }

  // Now, get that in seconds
  auto elapsed = (autonomy::commsgs::builtin_interfaces::Time::Now() - start_).Seconds();
  if (elapsed < period_) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::condition::PathExpiringTimerCondition>(
      "PathExpiringTimer");
}
