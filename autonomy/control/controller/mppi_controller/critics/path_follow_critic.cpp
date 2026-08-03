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

#include "autonomy/control/controller/mppi_controller/critics/path_follow_critic.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace critics {

void PathFollowCritic::initialize() {
  if (!options_) {
    AWARN << "Options not set, using defaults";
    power_ = 1;
    weight_ = 5.0f;
    threshold_to_consider_ = 1.4f;
    offset_from_furthest_ = 5;
    return;
  }

  // Load from proto options
  if (options_->has_path_follow_critic()) {
    const auto& critic = options_->path_follow_critic();
    enabled_ = critic.enabled();
    power_ = critic.cost_power();
    weight_ = static_cast<float>(critic.cost_weight());
    threshold_to_consider_ = static_cast<float>(critic.threshold_to_consider());
    offset_from_furthest_ = static_cast<size_t>(critic.offset_from_furthest());
  } else {
    enabled_ = true;
    power_ = 1;
    weight_ = 5.0f;                 // Default
    threshold_to_consider_ = 1.4f;  // Default
    offset_from_furthest_ = 5;      // Default
  }

  AINFO << "PathFollowCritic instantiated with " << power_ << " power and " << weight_ << " weight";
}

void PathFollowCritic::score(CriticData& data) {
  if (!enabled_) {
    return;
  }

  automsgs::msgs::geometry_msgs::Pose goal = tools::getCriticGoal(data, enforce_path_inversion_);

  if (data.path.x.size() < 2 ||
      tools::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose(), goal)) {
    return;
  }

  tools::setPathFurthestPointIfNotSet(data);
  tools::setPathCostsIfNotSet(data, costmap_ros_);
  const size_t path_size = data.path.x.size() - 1;

  auto offsetted_idx = std::min(*data.furthest_reached_path_point + offset_from_furthest_, path_size);

  // Drive to the first valid path point, in case of dynamic obstacles on path
  // we want to drive past it, not through it
  bool valid = false;
  while (!valid && offsetted_idx < path_size - 1) {
    valid = (*data.path_pts_valid)[offsetted_idx];
    if (!valid) {
      offsetted_idx++;
    }
  }

  const auto path_x = data.path.x(offsetted_idx);
  const auto path_y = data.path.y(offsetted_idx);

  const int&& rightmost_idx = data.trajectories.x.cols() - 1;
  const auto last_x = data.trajectories.x.col(rightmost_idx);
  const auto last_y = data.trajectories.y.col(rightmost_idx);

  const auto delta_x = last_x - path_x;
  const auto delta_y = last_y - path_y;
  if (power_ > 1u) {
    data.costs += (((delta_x.square() + delta_y.square()).sqrt()) * weight_).pow(power_);
  } else {
    data.costs += ((delta_x.square() + delta_y.square()).sqrt()) * weight_;
  }
}

}  // namespace critics
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

// Plugins
CLASS_LOADER_REGISTER_CLASS(autonomy::control::controller::mppi_controller::critics::PathFollowCritic,
                            autonomy::control::controller::mppi_controller::CriticFunction)