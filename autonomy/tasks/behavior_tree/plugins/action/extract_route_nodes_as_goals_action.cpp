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

#include "autonomy/tasks/behavior_tree/plugins/action/extract_route_nodes_as_goals_action.hpp"

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

ExtractRouteNodesAsGoals::ExtractRouteNodesAsGoals(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(name, conf) {}

BT::NodeStatus ExtractRouteNodesAsGoals::tick() {
  setStatus(BT::NodeStatus::RUNNING);

  proto::Route route;
  getInput("route", route);

  if (route.nodes_size() == 0) {
    return BT::NodeStatus::FAILURE;
  }

  commsgs::planning_msgs::Goals goals;
  goals.header = commsgs::std_msgs::FromProto(route.header());
  goals.goals.reserve(route.nodes_size());

  for (const auto& node : route.nodes()) {
    commsgs::geometry_msgs::PoseStamped goal;
    goal.header = goals.header;
    goal.pose.position.x = node.position().x();
    goal.pose.position.y = node.position().y();
    goal.pose.position.z = node.position().z();
    // Set default orientation (identity quaternion)
    goal.pose.orientation.w = 1.0;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goals.goals.push_back(goal);
  }

  setOutput("goals", goals);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::action::ExtractRouteNodesAsGoals>(
      "ExtractRouteNodesAsGoals");
}
