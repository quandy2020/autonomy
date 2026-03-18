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

#include "autonomy/control/controller/mppi_controller/critics/twirling_critic.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {
namespace critics {

void TwirlingCritic::initialize() {
  if (!options_) {
    AWARN << "Options not set, using defaults";
    power_ = 1;
    weight_ = 10.0f;
    return;
  }

  // Load from proto options
  if (options_->has_twirling_critic()) {
    const auto& critic = options_->twirling_critic();
    enabled_ = critic.enabled();
    power_ = critic.twirling_cost_power();
    weight_ = static_cast<float>(critic.twirling_cost_weight());
  } else {
    enabled_ = true;
    power_ = 1;
    weight_ = 10.0f;  // Default
  }

  AINFO << "TwirlingCritic instantiated with " << power_ << " power and " << weight_ << " weight.";
}

void TwirlingCritic::score(CriticData& data) {
  if (!enabled_) {
    return;
  }

  commsgs::geometry_msgs::Pose goal = tools::getCriticGoal(data, enforce_path_inversion_);

  if (tools::withinPositionGoalToleranceWithChecker(data.goal_checker, data.state.pose.pose, goal)) {
    return;
  }

  if (power_ > 1u) {
    data.costs += ((data.state.wz.abs().rowwise().mean()) * weight_).pow(power_).eval();
  } else {
    data.costs += ((data.state.wz.abs().rowwise().mean()) * weight_).eval();
  }
}

}  // namespace critics
}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

// Plugins
CLASS_LOADER_REGISTER_CLASS(autonomy::control::controller::mppi_controller::critics::TwirlingCritic,
                            autonomy::control::controller::mppi_controller::CriticFunction)