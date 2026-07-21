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

#include "autonomy/control/controller/teb_controller/tools/path_handler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "autolink/common/log.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {
namespace tools {

namespace {
using map::costmap_2d::utils::euclidean_distance;
}  // namespace

void PathHandler::initialize(
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
    std::shared_ptr<transform::Buffer> tf,
    const proto::TEBControllerOptions* options) {
  costmap_wrapper_ = std::move(costmap);
  tf_buffer_ = std::move(tf);
  options_ = options;
}

void PathHandler::setPath(const commsgs::planning_msgs::Path& path) {
  global_plan_ = path;
}

std::vector<commsgs::geometry_msgs::PoseStamped> PathHandler::transformGlobalPlan(
    const commsgs::geometry_msgs::PoseStamped& robot_pose) const {
  std::vector<commsgs::geometry_msgs::PoseStamped> transformed;
  if (global_plan_.poses.empty() || !costmap_wrapper_ || !options_) {
    return transformed;
  }

  const std::string global_frame = costmap_wrapper_->getGlobalFrameID();
  const std::string local_frame = costmap_wrapper_->getBaseFrameID();

  commsgs::geometry_msgs::PoseStamped robot_in_plan = robot_pose;
  if (robot_pose.header.frame_id != global_frame) {
    try {
      robot_in_plan = tf_buffer_->transform(robot_pose, global_frame);
    } catch (const std::exception& ex) {
      AWARN << "TEB PathHandler: TF transform failed: " << ex.what();
      return transformed;
    }
  }

  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const double d =
        euclidean_distance(global_plan_.poses[i], robot_in_plan);
    if (d < min_dist) {
      min_dist = d;
      closest_idx = i;
    }
  }

  const double max_dist =
      options_->max_global_plan_lookahead_dist() > 0.0
          ? options_->max_global_plan_lookahead_dist()
          : 3.0;
  double accumulated = 0.0;
  transformed.push_back(global_plan_.poses[closest_idx]);
  for (size_t i = closest_idx; i + 1 < global_plan_.poses.size(); ++i) {
    accumulated +=
        euclidean_distance(global_plan_.poses[i], global_plan_.poses[i + 1]);
    transformed.push_back(global_plan_.poses[i + 1]);
    if (accumulated >= max_dist) {
      break;
    }
  }

  if (transformed.size() < 2) {
    return transformed;
  }

  if (local_frame != global_frame) {
    for (size_t i = 0; i < transformed.size(); ++i) {
      commsgs::geometry_msgs::PoseStamped in = transformed[i];
      in.header.frame_id = global_frame;
      try {
        transformed[i] = tf_buffer_->transform(in, local_frame);
      } catch (const std::exception& ex) {
        AWARN << "TEB PathHandler: local transform failed: " << ex.what();
        break;
      }
    }
  }

  return transformed;
}

commsgs::geometry_msgs::PoseStamped PathHandler::getLocalGoal(
    const commsgs::geometry_msgs::PoseStamped& robot_pose) const {
  const auto plan = transformGlobalPlan(robot_pose);
  commsgs::geometry_msgs::PoseStamped goal;
  if (plan.empty()) {
    return goal;
  }
  goal.header.frame_id = costmap_wrapper_->getBaseFrameID();
  goal.pose.position.x = plan.back().pose.position.x;
  goal.pose.position.y = plan.back().pose.position.y;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.w = 1.0;
  if (options_->global_plan_overwrite_orientation() && plan.size() >= 2) {
    const auto& p0 = plan[plan.size() - 2].pose.position;
    const auto& p1 = plan.back().pose.position;
    const double yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
    goal.pose.orientation.z = std::sin(yaw * 0.5);
    goal.pose.orientation.w = std::cos(yaw * 0.5);
  }
  return goal;
}

}  // namespace tools
}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
