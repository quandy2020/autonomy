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

#include "autonomy/tasks/behavior_tree/plugins/action/truncate_path_action.hpp"

#include <cmath>

#include "autolink/common/log.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

TruncatePath::TruncatePath(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::ActionNodeBase(name, conf), distance_(1.0) {}

BT::NodeStatus TruncatePath::tick() {
  setStatus(BT::NodeStatus::RUNNING);
  getInput("distance", distance_);

  commsgs::planning_msgs::Path input_path;

  getInput("input_path", input_path);

  if (input_path.poses.empty()) {
    setOutput("output_path", input_path);
    return BT::NodeStatus::SUCCESS;
  }

  commsgs::geometry_msgs::PoseStamped final_pose = input_path.poses.back();

  double distance_to_goal = autonomy::map::costmap_2d::utils::euclidean_distance(input_path.poses.back(), final_pose);

  while (distance_to_goal < distance_ && input_path.poses.size() > 2) {
    input_path.poses.pop_back();
    distance_to_goal = autonomy::map::costmap_2d::utils::euclidean_distance(input_path.poses.back(), final_pose);
  }

  double dx = final_pose.pose.position.x - input_path.poses.back().pose.position.x;
  double dy = final_pose.pose.position.y - input_path.poses.back().pose.position.y;

  double final_angle = atan2(dy, dx);

  if (std::isnan(final_angle) || std::isinf(final_angle)) {
    AWARN << "Final angle is not valid while truncating path. Setting to "
             "0.0";
    final_angle = 0.0;
  }

  input_path.poses.back().pose.orientation = autonomy::map::costmap_2d::utils::OrientationAroundZAxis(final_angle);

  setOutput("output_path", input_path);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::action::TruncatePath>("TruncatePath");
}
