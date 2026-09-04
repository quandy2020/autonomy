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
 * @brief Maps a selected hypothesis / MINCO trajectory to differential-drive cmd.
 */

#pragma once

#include <array>
#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>

#include "autonomy/perception/proto/track_options.pb.h"
#include "autonomy/perception/track/common/types.hpp"

namespace autonomy::perception::track {

/**
 * @class autonomy::perception::track::VelocityCommandMapper
 * @brief Converts hypothesis / planar MINCO sample into a TwistStamped.
 */
class VelocityCommandMapper {
 public:
  VelocityCommandMapper();
  explicit VelocityCommandMapper(const proto::TrackOptions& options);

  void Configure(const proto::TrackOptions& options);

  /**
   * @brief Builds a velocity command for the selected hypothesis.
   * @param hypothesis Selected lattice hypothesis in the body frame.
   * @param frame_id Header frame (usually base_link).
   * @param[out] debug_path_xy_m Optional sampled path for visualization.
   */
  automsgs::msgs::geometry_msgs::TwistStamped ToVelocityCommand(
      const MotionPrimitiveHypothesis& hypothesis,
      const std::string& frame_id,
      std::vector<std::array<double, 2>>* debug_path_xy_m = nullptr) const;

 private:
  double max_linear_velocity_mps_{0.5};
  double max_yaw_rate_rps_{1.0};
  double desired_standoff_m_{1.5};
  double sample_horizon_s_{0.5};
  double range_error_gain_{0.8};
  double bearing_error_gain_{1.5};
};

}  // namespace autonomy::perception::track
