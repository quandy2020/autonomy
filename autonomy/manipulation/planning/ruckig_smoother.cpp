/*
 * Copyright 2026 The Openbot Authors
 *
 * Optional Ruckig trajectory smoother (FEATURES ruckig).
 */

#include "autonomy/manipulation/planning/time_parameterization.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "autonomy/common/logging.hpp"

#ifdef AUTONOMY_HAS_RUCKIG
#include <ruckig/ruckig.hpp>
#endif

namespace autonomy {
namespace manipulation {
namespace trajectory {

bool ApplyRuckig(core::RobotTrajectory* traj, const TimeParamOptions& options) {
  if (!traj || traj->waypoints.size() < 2) {
    return false;
  }

#ifdef AUTONOMY_HAS_RUCKIG
  const std::size_t dof = traj->waypoints.front().positions.size();
  if (dof == 0) {
    return false;
  }

  // Piecewise online Ruckig between consecutive waypoints.
  ruckig::Ruckig<ruckig::DynamicDOFs> otg(static_cast<int>(dof), 0.01);
  ruckig::InputParameter<ruckig::DynamicDOFs> input(static_cast<int>(dof));
  ruckig::OutputParameter<ruckig::DynamicDOFs> output(static_cast<int>(dof));

  const double vmax = std::max(1e-3, options.max_velocity);
  const double amax = std::max(1e-3, options.max_acceleration);
  for (std::size_t j = 0; j < dof; ++j) {
    input.max_velocity[j] = vmax;
    input.max_acceleration[j] = amax;
    input.max_jerk[j] = amax * 10.0;
  }

  core::RobotTrajectory smoothed;
  smoothed.waypoints.push_back(traj->waypoints.front());
  smoothed.time_from_start.push_back(0.0);
  double t_abs = 0.0;

  for (std::size_t i = 0; i + 1 < traj->waypoints.size(); ++i) {
    const auto& a = traj->waypoints[i].positions;
    const auto& b = traj->waypoints[i + 1].positions;
    for (std::size_t j = 0; j < dof; ++j) {
      input.current_position[j] = a[j];
      input.current_velocity[j] = 0.0;
      input.current_acceleration[j] = 0.0;
      input.target_position[j] = b[j];
      input.target_velocity[j] = 0.0;
      input.target_acceleration[j] = 0.0;
    }

    ruckig::Result result = otg.update(input, output);
    int guard = 0;
    while (result == ruckig::Result::Working && guard++ < 10000) {
      t_abs += 0.01;
      core::JointState wp;
      wp.names = traj->waypoints[i].names;
      wp.positions.resize(dof);
      for (std::size_t j = 0; j < dof; ++j) {
        wp.positions[j] = output.new_position[j];
      }
      smoothed.waypoints.push_back(wp);
      smoothed.time_from_start.push_back(t_abs);
      output.pass_to_input(input);
      result = otg.update(input, output);
    }
    if (result != ruckig::Result::Finished &&
        result != ruckig::Result::Working) {
      AWARN << "ApplyRuckig: segment failed, falling back to TOTG";
      return ApplyTotg(traj, options);
    }
  }

  *traj = std::move(smoothed);
  return true;
#else
  (void)options;
  AWARN << "ApplyRuckig: AUTONOMY_HAS_RUCKIG off; no-op (use ApplyTotg)";
  return true;
#endif
}

bool ApplyRuckig(core::RobotTrajectory* traj) {
  TimeParamOptions opts;
  return ApplyRuckig(traj, opts);
}

}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
