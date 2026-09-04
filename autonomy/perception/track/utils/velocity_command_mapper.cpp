/*
 * Copyright 2026 The Openbot Authors
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

/**
 * @file
 * @brief Hypothesis / MINCO trajectory to TwistStamped mapping.
 */

#include "autonomy/perception/track/utils/velocity_command_mapper.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/perception/track/common/constants.hpp"
#include "autonomy/perception/track/primitive/minco_trajectory.hpp"
#include "autonomy/perception/track/track_options.hpp"

namespace autonomy::perception::track {

VelocityCommandMapper::VelocityCommandMapper()
    : VelocityCommandMapper(DefaultOptions()) {}

VelocityCommandMapper::VelocityCommandMapper(
    const proto::TrackOptions& options) {
  Configure(options);
}

void VelocityCommandMapper::Configure(const proto::TrackOptions& options) {
  proto::TrackOptions sanitized = options;
  ApplyDefaults(&sanitized);
  max_linear_velocity_mps_ = sanitized.vel_max_mps();
  max_yaw_rate_rps_ = sanitized.wz_max_rps();
  desired_standoff_m_ = sanitized.follow_distance_m();
  sample_horizon_s_ = sanitized.minco_sample_horizon_s();
  range_error_gain_ = kRangeErrorGain;
  bearing_error_gain_ = kBearingErrorGain;
}

automsgs::msgs::geometry_msgs::TwistStamped
VelocityCommandMapper::ToVelocityCommand(
    const MotionPrimitiveHypothesis& hypothesis,
    const std::string& frame_id,
    std::vector<std::array<double, 2>>* debug_path_xy_m) const {
  automsgs::msgs::geometry_msgs::TwistStamped command;
  command.mutable_header()->set_frame_id(frame_id);
  if (debug_path_xy_m != nullptr) {
    debug_path_xy_m->clear();
  }

  if (hypothesis.uses_minco) {
    MincoTrajectory trajectory;
    if (trajectory.Solve(hypothesis.minco)) {
      const double sample_t =
          std::min(sample_horizon_s_, trajectory.total_time_s());
      const Eigen::Vector2d velocity = trajectory.Velocity(sample_t);
      double linear_velocity_mps = 0.0;
      double yaw_rate_rps = 0.0;
      SampleDifferentialDriveCommand(velocity, max_linear_velocity_mps_,
                                     max_yaw_rate_rps_, &linear_velocity_mps,
                                     &yaw_rate_rps);
      command.mutable_twist()->mutable_linear()->set_x(linear_velocity_mps);
      command.mutable_twist()->mutable_angular()->set_z(yaw_rate_rps);
      if (debug_path_xy_m != nullptr) {
        *debug_path_xy_m = trajectory.SamplePath(20);
      }
      return command;
    }
  }

  const double range_m =
      std::hypot(hypothesis.terminal_x_m, hypothesis.terminal_y_m);
  const double bearing_rad =
      std::atan2(hypothesis.terminal_y_m, hypothesis.terminal_x_m);
  const double range_error_m = range_m - desired_standoff_m_;

  double linear_velocity_mps = hypothesis.linear_velocity_mps;
  if (std::abs(linear_velocity_mps) < kNearZeroVelocityEpsilon) {
    linear_velocity_mps = range_error_gain_ * range_error_m;
  }
  double yaw_rate_rps = hypothesis.yaw_rate_rps;
  if (std::abs(yaw_rate_rps) < kNearZeroVelocityEpsilon) {
    yaw_rate_rps = bearing_error_gain_ * bearing_rad;
  }

  linear_velocity_mps = std::clamp(
      linear_velocity_mps, -max_linear_velocity_mps_, max_linear_velocity_mps_);
  yaw_rate_rps =
      std::clamp(yaw_rate_rps, -max_yaw_rate_rps_, max_yaw_rate_rps_);
  if (linear_velocity_mps < 0.0 &&
      range_m > desired_standoff_m_ * kReverseInhibitStandoffFraction) {
    linear_velocity_mps = 0.0;
  }

  command.mutable_twist()->mutable_linear()->set_x(linear_velocity_mps);
  command.mutable_twist()->mutable_angular()->set_z(yaw_rate_rps);
  if (debug_path_xy_m != nullptr) {
    debug_path_xy_m->push_back({0.0, 0.0});
    debug_path_xy_m->push_back(
        {hypothesis.terminal_x_m, hypothesis.terminal_y_m});
  }
  return command;
}

}  // namespace autonomy::perception::track
