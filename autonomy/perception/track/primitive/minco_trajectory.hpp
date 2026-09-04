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
 * @brief Planar 2-piece MINCO_S3NU trajectory (YOPO-MINCO adapted to ground robots).
 */

#pragma once

#include <array>
#include <vector>

#include <Eigen/Dense>

#include "autonomy/perception/track/common/types.hpp"

namespace autonomy::perception::track {

/**
 * @class autonomy::perception::track::MincoTrajectory
 * @brief Solves and evaluates a 2-piece planar quintic MINCO trajectory.
 *
 * Compatible with YOPO-MINCO's MincoTraj layout, reduced to spatial dim = 2:
 *   head_pva / tail_pva rows = (pos, vel, acc), cols = (x, y)
 *   inner waypoint = (x, y)
 *   durations = (T0, T1)
 */
class MincoTrajectory {
 public:
  using Matrix12 = Eigen::Matrix<double, 12, 12>;
  using Matrix12x2 = Eigen::Matrix<double, 12, 2>;
  using Coeffs = Eigen::Matrix<double, 12, 2>;

  /**
   * @brief Solves polynomial coefficients from planar boundary conditions.
   * @return False when durations are non-positive or the system is singular.
   */
  bool Solve(const MincoBoundary& boundary);

  double total_time_s() const { return total_time_s_; }
  const std::array<double, 2>& durations_s() const { return durations_s_; }
  bool is_solved() const { return is_solved_; }

  /**
   * @brief Evaluates position at absolute time |time_s| (clamped to [0, T]).
   */
  Eigen::Vector2d Position(double time_s) const;

  /**
   * @brief Evaluates velocity at absolute time |time_s|.
   */
  Eigen::Vector2d Velocity(double time_s) const;

  /**
   * @brief Evaluates acceleration at absolute time |time_s|.
   */
  Eigen::Vector2d Acceleration(double time_s) const;

  /**
   * @brief Uniformly samples positions for visualization / Path publish.
   */
  std::vector<std::array<double, 2>> SamplePath(int sample_count) const;

 private:
  Eigen::Vector2d Evaluate(double time_s, int derivative) const;
  static Matrix12 BuildConstraintMatrix(double duration0, double duration1);

  Coeffs coefficients_ = Coeffs::Zero();
  std::array<double, 2> durations_s_{{1.0, 1.0}};
  double total_time_s_{2.0};
  bool is_solved_{false};
};

/**
 * @brief Maps a body-frame velocity sample to differential-drive cmd rates.
 */
void SampleDifferentialDriveCommand(const Eigen::Vector2d& velocity_mps,
                                    double max_linear_velocity_mps,
                                    double max_yaw_rate_rps,
                                    double* linear_velocity_mps,
                                    double* yaw_rate_rps);

}  // namespace autonomy::perception::track
