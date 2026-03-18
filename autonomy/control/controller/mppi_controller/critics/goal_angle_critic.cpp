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

#include "autonomy/control/controller/mppi_controller/critics/goal_angle_critic.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace critics {

void GoalAngleCritic::initialize() {
  if (!options_) {
    AWARN << "Options not set, using defaults";
    power_ = 1;
    weight_ = 3.0f;
    threshold_to_consider_ = 0.5f;
    return;
  }

  // Load from proto options
  if (options_->has_goal_angle_critic()) {
    const auto& critic = options_->goal_angle_critic();
    enabled_ = critic.enabled();
    power_ = critic.cost_power();
    weight_ = static_cast<float>(critic.cost_weight());
    threshold_to_consider_ = static_cast<float>(critic.threshold_to_consider());
  } else {
    enabled_ = true;
    power_ = 1;
    weight_ = 3.0f;                 // Default
    threshold_to_consider_ = 0.5f;  // Default
  }

  AINFO << "GoalAngleCritic instantiated with " << power_ << " power, " << weight_ << " weight, and "
        << threshold_to_consider_ << " angular threshold.";
}

void GoalAngleCritic::score(CriticData& data) {
  if (!enabled_) {
    return;
  }

  commsgs::geometry_msgs::Pose goal = tools::getCriticGoal(data, enforce_path_inversion_);

  if (!tools::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, goal)) {
    return;
  }

  double goal_yaw = autonomy::transform::tf2::getYaw(goal.orientation);

  if (power_ > 1u) {
    data.costs +=
        (((tools::shortest_angular_distance(data.trajectories.yaws, goal_yaw).abs()).rowwise().mean()) * weight_)
            .pow(power_)
            .eval();
  } else {
    data.costs +=
        (((tools::shortest_angular_distance(data.trajectories.yaws, goal_yaw).abs()).rowwise().mean()) * weight_)
            .eval();
  }
}

}  // namespace critics
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

// Plugins
CLASS_LOADER_REGISTER_CLASS(autonomy::control::controller::mppi_controller::critics::GoalAngleCritic,
                            autonomy::control::controller::mppi_controller::CriticFunction)