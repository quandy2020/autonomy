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

#include "autonomy/tasks/behavior_tree/plugins/action/remove_in_collision_goals_action.hpp"

#include <cmath>

#include "autolink/common/log.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/tasks/navigator/proto/msg.pb.h"
#include "autonomy/tasks/navigator/proto/srv.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

namespace {
// Helper function to find matching goal in waypoint statuses
int findNextMatchingGoalInWaypointStatuses(const std::vector<proto::WaypointStatus>& waypoint_statuses,
                                           const commsgs::geometry_msgs::PoseStamped& goal) {
  constexpr double tolerance = 0.01;  // 1cm tolerance for position matching

  for (size_t i = 0; i < waypoint_statuses.size(); ++i) {
    const auto& waypoint = waypoint_statuses[i];
    if (waypoint.has_waypoint_pose()) {
      // Convert proto pose to commsgs pose for comparison
      commsgs::geometry_msgs::PoseStamped waypoint_pose = commsgs::geometry_msgs::FromProto(waypoint.waypoint_pose());

      // Compare positions (ignore frame_id and timestamp for matching)
      double dx = waypoint_pose.pose.position.x - goal.pose.position.x;
      double dy = waypoint_pose.pose.position.y - goal.pose.position.y;
      double dist = std::hypot(dx, dy);

      if (dist < tolerance) {
        return static_cast<int>(i);
      }
    }
  }
  return -1;
}
}  // namespace

RemoveInCollisionGoals::RemoveInCollisionGoals(const std::string& service_node_name, const BT::NodeConfiguration& conf)
    : BtServiceNode<proto::GetCosts>(service_node_name, conf, "/global_costmap/get_cost_global_costmap") {}

void RemoveInCollisionGoals::on_tick() {
  getInput("use_footprint", use_footprint_);
  getInput("cost_threshold", cost_threshold_);
  getInput("input_goals", input_goals_);
  getInput("consider_unknown_as_obstacle", consider_unknown_as_obstacle_);

  if (input_goals_.goals.empty()) {
    setOutput("output_goals", input_goals_);
    should_send_request_ = false;
    return;
  }
  request_ = std::make_shared<proto::GetCosts::Request>();
  request_->set_use_footprint(use_footprint_);

  for (const auto& goal : input_goals_.goals) {
    auto* pose = request_->mutable_poses()->Add();
    *pose = commsgs::geometry_msgs::ToProto(goal);
  }
}

BT::NodeStatus RemoveInCollisionGoals::on_completion(std::shared_ptr<proto::GetCosts::Response> response) {
  if (!response->success()) {
    AERROR << "GetCosts service call failed";
    setOutput("output_goals", input_goals_);
    return BT::NodeStatus::FAILURE;
  }

  // get the `waypoint_statuses` vector
  std::vector<proto::WaypointStatus> waypoint_statuses;
  auto waypoint_statuses_get_res = getInput("input_waypoint_statuses", waypoint_statuses);
  if (!waypoint_statuses_get_res) {
    AERROR << "Missing [input_waypoint_statuses] port input!";
  }

  commsgs::planning_msgs::Goals valid_goal_poses;
  valid_goal_poses.header = input_goals_.header;
  for (size_t i = 0; i < response->costs_size(); ++i) {
    float cost = response->costs(i);
    if ((cost == 255 && !consider_unknown_as_obstacle_) || cost < cost_threshold_) {
      valid_goal_poses.goals.push_back(input_goals_.goals[i]);
    } else if (waypoint_statuses_get_res) {
      // Find matching goal in waypoint_statuses by position
      int cur_waypoint_index = findNextMatchingGoalInWaypointStatuses(waypoint_statuses, input_goals_.goals[i]);
      if (cur_waypoint_index == -1) {
        AERROR << "Failed to find matching goal in waypoint_statuses";
        return BT::NodeStatus::FAILURE;
      }
      waypoint_statuses[cur_waypoint_index].set_waypoint_status(proto::WaypointStatusType::WAYPOINT_STATUS_SKIPPED);
    }
  }
  // Inform if all goals have been removed
  if (valid_goal_poses.goals.empty()) {
    AINFO << "All goals are in collision and have been removed from the list";
  }
  setOutput("output_goals", valid_goal_poses);
  // set `waypoint_statuses` output
  setOutput("output_waypoint_statuses", waypoint_statuses);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::action::RemoveInCollisionGoals>(
      "RemoveInCollisionGoals");
}
