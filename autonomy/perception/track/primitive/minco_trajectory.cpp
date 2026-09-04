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
 * @brief Planar 2-piece MINCO_S3NU solver / evaluator.
 */

#include "autonomy/perception/track/primitive/minco_trajectory.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/perception/track/common/constants.hpp"

namespace autonomy::perception::track {

MincoTrajectory::Matrix12 MincoTrajectory::BuildConstraintMatrix(
    double duration0, double duration1) {
  Matrix12 A = Matrix12::Zero();
  // Head pos / vel / acc.
  A(0, 0) = 1.0;
  A(1, 1) = 1.0;
  A(2, 2) = 2.0;

  const double t0 = duration0;
  const double t0_2 = t0 * t0;
  const double t0_3 = t0_2 * t0;
  const double t0_4 = t0_3 * t0;
  const double t0_5 = t0_4 * t0;
  // Piece 0 at T0: inner waypoint + C4 continuity.
  A(3, 0) = 1.0;
  A(3, 1) = t0;
  A(3, 2) = t0_2;
  A(3, 3) = t0_3;
  A(3, 4) = t0_4;
  A(3, 5) = t0_5;
  A.row(4).head<6>() = A.row(3).head<6>();
  A(4, 6) = -1.0;
  A(5, 1) = 1.0;
  A(5, 2) = 2.0 * t0;
  A(5, 3) = 3.0 * t0_2;
  A(5, 4) = 4.0 * t0_3;
  A(5, 5) = 5.0 * t0_4;
  A(5, 7) = -1.0;
  A(6, 2) = 2.0;
  A(6, 3) = 6.0 * t0;
  A(6, 4) = 12.0 * t0_2;
  A(6, 5) = 20.0 * t0_3;
  A(6, 8) = -2.0;
  A(7, 3) = 6.0;
  A(7, 4) = 24.0 * t0;
  A(7, 5) = 60.0 * t0_2;
  A(7, 9) = -6.0;
  A(8, 4) = 24.0;
  A(8, 5) = 120.0 * t0;
  A(8, 10) = -24.0;

  const double t1 = duration1;
  const double t1_2 = t1 * t1;
  const double t1_3 = t1_2 * t1;
  const double t1_4 = t1_3 * t1;
  const double t1_5 = t1_4 * t1;
  // Piece 1 at T1: tail pos / vel / acc.
  A(9, 6) = 1.0;
  A(9, 7) = t1;
  A(9, 8) = t1_2;
  A(9, 9) = t1_3;
  A(9, 10) = t1_4;
  A(9, 11) = t1_5;
  A(10, 7) = 1.0;
  A(10, 8) = 2.0 * t1;
  A(10, 9) = 3.0 * t1_2;
  A(10, 10) = 4.0 * t1_3;
  A(10, 11) = 5.0 * t1_4;
  A(11, 8) = 2.0;
  A(11, 9) = 6.0 * t1;
  A(11, 10) = 12.0 * t1_2;
  A(11, 11) = 20.0 * t1_3;
  return A;
}

bool MincoTrajectory::Solve(const MincoBoundary& boundary) {
  is_solved_ = false;
  durations_s_ = boundary.durations_s;
  durations_s_[0] = std::max(durations_s_[0], kMincoMinPieceDurationS);
  durations_s_[1] = std::max(durations_s_[1], kMincoMinPieceDurationS);
  total_time_s_ = durations_s_[0] + durations_s_[1];

  Matrix12x2 b = Matrix12x2::Zero();
  b(0, 0) = boundary.head.position_x_m;
  b(0, 1) = boundary.head.position_y_m;
  b(1, 0) = boundary.head.velocity_x_mps;
  b(1, 1) = boundary.head.velocity_y_mps;
  b(2, 0) = boundary.head.acceleration_x_mps2;
  b(2, 1) = boundary.head.acceleration_y_mps2;
  b(3, 0) = boundary.inner_x_m;
  b(3, 1) = boundary.inner_y_m;
  b(9, 0) = boundary.tail.position_x_m;
  b(9, 1) = boundary.tail.position_y_m;
  b(10, 0) = boundary.tail.velocity_x_mps;
  b(10, 1) = boundary.tail.velocity_y_mps;
  b(11, 0) = boundary.tail.acceleration_x_mps2;
  b(11, 1) = boundary.tail.acceleration_y_mps2;

  const Matrix12 A =
      BuildConstraintMatrix(durations_s_[0], durations_s_[1]);
  Eigen::FullPivLU<Matrix12> lu(A);
  if (!lu.isInvertible()) {
    return false;
  }
  coefficients_ = lu.solve(b);
  is_solved_ = true;
  return true;
}

Eigen::Vector2d MincoTrajectory::Evaluate(double time_s, int derivative) const {
  if (!is_solved_) {
    return Eigen::Vector2d::Zero();
  }
  const double clamped =
      std::clamp(time_s, 0.0, std::max(0.0, total_time_s_ - 1e-9));
  const int piece = clamped >= durations_s_[0] ? 1 : 0;
  const double local_t = clamped - (piece == 0 ? 0.0 : durations_s_[0]);
  const int offset = piece * 6;
  Eigen::Matrix<double, 6, 1> powers;
  if (derivative == 0) {
    powers << 1.0, local_t, local_t * local_t, std::pow(local_t, 3),
        std::pow(local_t, 4), std::pow(local_t, 5);
  } else if (derivative == 1) {
    powers << 0.0, 1.0, 2.0 * local_t, 3.0 * local_t * local_t,
        4.0 * std::pow(local_t, 3), 5.0 * std::pow(local_t, 4);
  } else {
    powers << 0.0, 0.0, 2.0, 6.0 * local_t, 12.0 * local_t * local_t,
        20.0 * std::pow(local_t, 3);
  }
  return coefficients_.block<6, 2>(offset, 0).transpose() * powers;
}

Eigen::Vector2d MincoTrajectory::Position(double time_s) const {
  return Evaluate(time_s, 0);
}

Eigen::Vector2d MincoTrajectory::Velocity(double time_s) const {
  return Evaluate(time_s, 1);
}

Eigen::Vector2d MincoTrajectory::Acceleration(double time_s) const {
  return Evaluate(time_s, 2);
}

std::vector<std::array<double, 2>> MincoTrajectory::SamplePath(
    int sample_count) const {
  std::vector<std::array<double, 2>> path;
  if (!is_solved_ || sample_count <= 1) {
    return path;
  }
  path.reserve(static_cast<size_t>(sample_count));
  for (int index = 0; index < sample_count; ++index) {
    const double time_s =
        total_time_s_ * static_cast<double>(index) /
        static_cast<double>(sample_count - 1);
    const Eigen::Vector2d position = Position(time_s);
    path.push_back({position.x(), position.y()});
  }
  return path;
}

void SampleDifferentialDriveCommand(const Eigen::Vector2d& velocity_mps,
                                    double max_linear_velocity_mps,
                                    double max_yaw_rate_rps,
                                    double* linear_velocity_mps,
                                    double* yaw_rate_rps) {
  if (linear_velocity_mps == nullptr || yaw_rate_rps == nullptr) {
    return;
  }
  const double speed = velocity_mps.norm();
  *linear_velocity_mps =
      std::clamp(speed, 0.0, std::max(0.0, max_linear_velocity_mps));
  if (speed < kNearZeroVelocityEpsilon) {
    *yaw_rate_rps = 0.0;
    return;
  }
  // Pure-pursuit style: angular rate from velocity heading vs body +x.
  const double bearing_rad = std::atan2(velocity_mps.y(), velocity_mps.x());
  *yaw_rate_rps =
      std::clamp(bearing_rad * kBearingErrorGain, -max_yaw_rate_rps,
                 max_yaw_rate_rps);
}

}  // namespace autonomy::perception::track
