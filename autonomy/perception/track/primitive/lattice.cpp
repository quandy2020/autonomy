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
 * @brief Planar yaw×range lattice construction and decoding.
 */

#include "autonomy/perception/track/primitive/lattice.hpp"

#include <algorithm>

#include "autonomy/perception/track/common/constants.hpp"

namespace autonomy::perception::track {

Lattice::Lattice(const proto::TrackOptions& options) { Configure(options); }

void Lattice::Configure(const proto::TrackOptions& options) {
  horizontal_bin_count_ = std::max(1, options.horizon_num());
  vertical_bin_count_ = std::max(1, options.vertical_num());
  camera_horizontal_field_of_view_rad_ =
      (options.horizon_camera_fov_deg() > 0
           ? options.horizon_camera_fov_deg()
           : defaults::kCameraHorizontalFieldOfViewDeg) *
      M_PI / 180.0;
  planning_horizon_m_ = options.radio_range_m() > 0
                            ? options.radio_range_m()
                            : defaults::kPlanningHorizonM;
  max_linear_velocity_mps_ = options.vel_max_mps() > 0
                                 ? options.vel_max_mps()
                                 : defaults::kMaxLinearVelocityMps;
  max_yaw_rate_rps_ = options.wz_max_rps() > 0 ? options.wz_max_rps()
                                               : defaults::kMaxYawRateRps;
  yaw_bin_half_width_rad_ =
      horizontal_bin_count_ > 1
          ? 0.5 * camera_horizontal_field_of_view_rad_ /
                static_cast<double>(horizontal_bin_count_)
          : 0.5 * camera_horizontal_field_of_view_rad_;
  RebuildAnchors();
}

// ---------------------------------------------------------------------------
// Anchor generation
// ---------------------------------------------------------------------------
void Lattice::RebuildAnchors() {
  anchors_.clear();
  anchors_.reserve(
      static_cast<size_t>(horizontal_bin_count_ * vertical_bin_count_));
  const double yaw_step =
      horizontal_bin_count_ > 1
          ? camera_horizontal_field_of_view_rad_ /
                static_cast<double>(horizontal_bin_count_)
          : 0.0;
  int index = 0;
  for (int v = 0; v < vertical_bin_count_; ++v) {
    (void)v;
    for (int h = 0; h < horizontal_bin_count_; ++h) {
      LatticeAnchor anchor;
      anchor.index = index++;
      anchor.yaw_rad =
          -0.5 * yaw_step * (horizontal_bin_count_ - 1) + h * yaw_step;
      anchor.range_m = planning_horizon_m_;
      anchor.position_x_m = std::cos(anchor.yaw_rad) * anchor.range_m;
      anchor.position_y_m = std::sin(anchor.yaw_rad) * anchor.range_m;
      anchors_.push_back(anchor);
    }
  }
}

// ---------------------------------------------------------------------------
// Network offset → body-frame terminal state
// ---------------------------------------------------------------------------
void Lattice::DecodeTerminalState(
    int lattice_index, double normalized_yaw_offset,
    double normalized_range_offset, double normalized_linear_velocity,
    double normalized_yaw_rate, double* terminal_x_m, double* terminal_y_m,
    double* linear_velocity_mps, double* yaw_rate_rps) const {
  const LatticeAnchor& reference = anchor(lattice_index);
  const double yaw_rad =
      reference.yaw_rad + normalized_yaw_offset * yaw_bin_half_width_rad_;
  const double range_m =
      (normalized_range_offset + 1.0) * planning_horizon_m_;
  if (terminal_x_m != nullptr) {
    *terminal_x_m = std::cos(yaw_rad) * range_m;
  }
  if (terminal_y_m != nullptr) {
    *terminal_y_m = std::sin(yaw_rad) * range_m;
  }
  if (linear_velocity_mps != nullptr) {
    *linear_velocity_mps =
        normalized_linear_velocity * max_linear_velocity_mps_;
  }
  if (yaw_rate_rps != nullptr) {
    *yaw_rate_rps = normalized_yaw_rate * max_yaw_rate_rps_;
  }
}

}  // namespace autonomy::perception::track
