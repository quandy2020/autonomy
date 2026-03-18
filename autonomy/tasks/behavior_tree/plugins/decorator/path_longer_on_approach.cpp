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

#include "autonomy/tasks/behavior_tree/plugins/decorator/path_longer_on_approach.hpp"

#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace decorator {

PathLongerOnApproach::PathLongerOnApproach(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::DecoratorNode(name, conf) {
  node_ = config().blackboard->get<std::shared_ptr<autolink::Node>>("node");
}

bool PathLongerOnApproach::isPathUpdated(commsgs::planning_msgs::Path& new_path,
                                         commsgs::planning_msgs::Path& old_path) {
  if (old_path.poses.size() == 0 || new_path.poses.size() == 0) {
    return false;
  }
  if (new_path.poses.size() == old_path.poses.size()) {
    return false;
  }
  // Compare positions manually
  const auto& old_pos = old_path.poses.back().pose.position;
  const auto& new_pos = new_path.poses.back().pose.position;
  return old_pos.x == new_pos.x && old_pos.y == new_pos.y && old_pos.z == new_pos.z;
}

bool PathLongerOnApproach::isRobotInGoalProximity(commsgs::planning_msgs::Path& old_path, double& prox_leng) {
  return autonomy::map::costmap_2d::utils::calculate_path_length(old_path, 0) < prox_leng;
}

bool PathLongerOnApproach::isNewPathLonger(commsgs::planning_msgs::Path& new_path,
                                           commsgs::planning_msgs::Path& old_path, double& length_factor) {
  return autonomy::map::costmap_2d::utils::calculate_path_length(new_path, 0) >
         length_factor * autonomy::map::costmap_2d::utils::calculate_path_length(old_path, 0);
}

inline BT::NodeStatus PathLongerOnApproach::tick() {
  getInput("path", new_path_);
  getInput("prox_len", prox_len_);
  getInput("length_factor", length_factor_);

  if (first_time_ == false) {
    if (old_path_.poses.empty() || new_path_.poses.empty()) {
      first_time_ = true;
    } else {
      // Compare poses manually
      const auto& old_pose = old_path_.poses.back().pose;
      const auto& new_pose = new_path_.poses.back().pose;
      bool poses_different =
          old_pose.position.x != new_pose.position.x || old_pose.position.y != new_pose.position.y ||
          old_pose.position.z != new_pose.position.z || old_pose.orientation.x != new_pose.orientation.x ||
          old_pose.orientation.y != new_pose.orientation.y || old_pose.orientation.z != new_pose.orientation.z ||
          old_pose.orientation.w != new_pose.orientation.w;
      if (poses_different) {
        first_time_ = true;
      }
    }
  }
  setStatus(BT::NodeStatus::RUNNING);

  // Check if the path is updated and valid, compare the old and the new path
  // length, given the goal proximity and check if the new path is longer
  if (isPathUpdated(new_path_, old_path_) && isRobotInGoalProximity(old_path_, prox_len_) &&
      isNewPathLonger(new_path_, old_path_, length_factor_) && !first_time_) {
    const BT::NodeStatus child_state = child_node_->executeTick();
    switch (child_state) {
      case BT::NodeStatus::SKIPPED:
      case BT::NodeStatus::RUNNING:
        return child_state;
      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::FAILURE:
        old_path_ = new_path_;
        resetChild();
        return child_state;
      default:
        old_path_ = new_path_;
        return BT::NodeStatus::FAILURE;
    }
  }
  old_path_ = new_path_;
  first_time_ = false;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace decorator
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::decorator::PathLongerOnApproach>(
      "PathLongerOnApproach");
}
