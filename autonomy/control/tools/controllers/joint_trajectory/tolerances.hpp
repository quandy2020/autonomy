// Copyright 2013 PAL Robotics S.L.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/// \author Adolfo Rodriguez Tsouroukdissian

// Ported into autonomy::control::tools::controllers
// Self-contained tolerance structs + checkers only (no ROS params / actions).

#ifndef AUTONOMY_CONTROL_TOOLS_CONTROLLERS_JOINT_TRAJECTORY__TOLERANCES_HPP_
#define AUTONOMY_CONTROL_TOOLS_CONTROLLERS_JOINT_TRAJECTORY__TOLERANCES_HPP_

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "autonomy/control/tools/controllers/joint_trajectory/trajectory_types.hpp"

namespace autonomy {
namespace control {
namespace tools {
namespace controllers {
namespace joint_trajectory
{

struct StateTolerances
{
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
};

struct SegmentTolerances
{
  explicit SegmentTolerances(size_t size = 0) : state_tolerance(size), goal_state_tolerance(size) {}

  std::vector<StateTolerances> state_tolerance;
  std::vector<StateTolerances> goal_state_tolerance;
  double goal_time_tolerance = 0.0;
};

inline double resolve_tolerance_source(double default_value, double goal_value)
{
  constexpr double ERASE_VALUE = -1.0;
  auto is_erase_value = [=](double value) {
    return std::fabs(value - ERASE_VALUE) < std::numeric_limits<float>::epsilon();
  };

  if (goal_value > 0.0)
  {
    return goal_value;
  }
  if (is_erase_value(goal_value))
  {
    return 0.0;
  }
  if (goal_value < 0.0)
  {
    throw std::runtime_error("Illegal tolerance value.");
  }
  return default_value;
}

inline bool check_state_tolerance_per_joint(
  const TrajectoryPoint & state_error, size_t joint_idx, const StateTolerances & state_tolerance)
{
  using std::abs;
  const double error_position = state_error.positions[joint_idx];
  const double error_velocity =
    state_error.velocities.empty() ? 0.0 : state_error.velocities[joint_idx];
  const double error_acceleration =
    state_error.accelerations.empty() ? 0.0 : state_error.accelerations[joint_idx];

  return !(state_tolerance.position > 0.0 && abs(error_position) > state_tolerance.position) &&
         !(state_tolerance.velocity > 0.0 && abs(error_velocity) > state_tolerance.velocity) &&
         !(
           state_tolerance.acceleration > 0.0 &&
           abs(error_acceleration) > state_tolerance.acceleration);
}

}  // namespace joint_trajectory
}  // namespace controllers
}  // namespace tools
}  // namespace control
}  // namespace autonomy

#endif  // AUTONOMY_CONTROL_TOOLS_CONTROLLERS_JOINT_TRAJECTORY__TOLERANCES_HPP_
