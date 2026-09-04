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
 * @brief Planar yaw×range motion-primitive lattice for ground-robot YOPO.
 */

#pragma once

#include <cmath>
#include <vector>

#include "autonomy/perception/proto/track_options.pb.h"

namespace autonomy::perception::track {

/**
 * @struct autonomy::perception::track::LatticeAnchor
 * @brief One anchor ray in the robot body frame.
 */
struct LatticeAnchor {
  // Dense index in row-major (vertical × horizontal) order.
  int index{0};

  // Body-frame heading of the ray [rad].
  double yaw_rad{0.0};

  // Nominal planning horizon along the ray [m].
  double range_m{0.0};

  // Anchor terminal position x [m].
  double position_x_m{0.0};

  // Anchor terminal position y [m].
  double position_y_m{0.0};
};

/**
 * @class autonomy::perception::track::Lattice
 * @brief Uniform yaw bins covering the depth camera field of view (vertical bins usually 1).
 */
class Lattice {
 public:
  /**
   * @brief Default-constructs an unconfigured lattice.
   */
  Lattice() = default;

  /**
   * @brief Constructs and configures the lattice from tracking options.
   * @param options Proto options providing field of view, bin counts, and limits.
   */
  explicit Lattice(const proto::TrackOptions& options);

  /**
   * @brief Rebuilds anchors from |options|.
   * @param options Proto options providing field of view, bin counts, and limits.
   */
  void Configure(const proto::TrackOptions& options);

  /**
   * @brief Number of lattice anchors.
   */
  int size() const { return static_cast<int>(anchors_.size()); }

  /**
   * @brief Horizontal bin count.
   */
  int horizontal_bin_count() const { return horizontal_bin_count_; }

  /**
   * @brief Vertical bin count (1 for planar ground following).
   */
  int vertical_bin_count() const { return vertical_bin_count_; }

  /**
   * @brief Nominal planning horizon [m].
   */
  double planning_horizon_m() const { return planning_horizon_m_; }

  /**
   * @brief Half-width of the per-bin yaw offset used when decoding [-1, 1].
   */
  double yaw_bin_half_width_rad() const { return yaw_bin_half_width_rad_; }

  /**
   * @brief All lattice anchors.
   */
  const std::vector<LatticeAnchor>& anchors() const { return anchors_; }

  /**
   * @brief Anchor at |index|; throws if out of range.
   * @param index Lattice index.
   */
  const LatticeAnchor& anchor(int index) const {
    return anchors_.at(static_cast<size_t>(index));
  }

  /**
   * @brief Decodes normalized network offsets into a body-frame terminal state.
   * @param lattice_index Anchor index.
   * @param normalized_yaw_offset Yaw offset in [-1, 1].
   * @param normalized_range_offset Range offset in [-1, 1] → [0, 2·horizon].
   * @param normalized_linear_velocity Linear velocity in [-1, 1].
   * @param normalized_yaw_rate Yaw rate in [-1, 1].
   * @param[out] terminal_x_m Body-frame terminal x [m] (optional).
   * @param[out] terminal_y_m Body-frame terminal y [m] (optional).
   * @param[out] linear_velocity_mps Body-frame vx [m/s] (optional).
   * @param[out] yaw_rate_rps Yaw rate [rad/s] (optional).
   */
  void DecodeTerminalState(int lattice_index, double normalized_yaw_offset,
                           double normalized_range_offset,
                           double normalized_linear_velocity,
                           double normalized_yaw_rate, double* terminal_x_m,
                           double* terminal_y_m, double* linear_velocity_mps,
                           double* yaw_rate_rps) const;

 private:
  /**
   * @brief Regenerates |anchors_| from the current field of view / bin parameters.
   */
  void RebuildAnchors();

  // Horizontal primitive count.
  int horizontal_bin_count_{5};

  // Vertical primitive count (planar default: 1).
  int vertical_bin_count_{1};

  // Depth-camera horizontal field of view [rad].
  double camera_horizontal_field_of_view_rad_{M_PI / 2.0};

  // Nominal planning horizon [m].
  double planning_horizon_m_{3.0};

  // Max linear speed used to denormalize linear velocity [m/s].
  double max_linear_velocity_mps_{0.5};

  // Max yaw rate used to denormalize yaw rate [rad/s].
  double max_yaw_rate_rps_{1.0};

  // Half yaw bin width for offset decoding [rad].
  double yaw_bin_half_width_rad_{0.0};

  // Materialized anchors.
  std::vector<LatticeAnchor> anchors_;
};

}  // namespace autonomy::perception::track
