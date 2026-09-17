/*
 * Copyright 2026 The Openbot Authors
 *
 * Optional Ruckig trajectory smoother (FEATURES ruckig).
 */

#include "autonomy/manipulation/planner/pipeline/time_parameterization.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/common/joint_state_util.hpp"

#ifdef AUTONOMY_HAS_RUCKIG
#include <ruckig/ruckig.hpp>
#endif

namespace autonomy {
namespace manipulation {
namespace trajectory {

bool ApplyRuckig(core::RobotTrajectory* traj, const TimeParamOptions& options) {
  if (!traj || traj->points_size() < 2) {
    return false;
  }

#ifdef AUTONOMY_HAS_RUCKIG
  const std::size_t dof =
      static_cast<std::size_t>(traj->points(0).positions_size());
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
  AddTrajectoryPoint(&smoothed, MakeJointStateFromPoint(*traj, 0), 0.0);
  double t_abs = 0.0;

  std::vector<std::string> names(traj->joint_names().begin(),
                                 traj->joint_names().end());

  for (int i = 0; i + 1 < traj->points_size(); ++i) {
    const auto& a = traj->points(i);
    const auto& b = traj->points(i + 1);
    for (std::size_t j = 0; j < dof; ++j) {
      input.current_position[j] = a.positions(static_cast<int>(j));
      input.current_velocity[j] = 0.0;
      input.current_acceleration[j] = 0.0;
      input.target_position[j] = b.positions(static_cast<int>(j));
      input.target_velocity[j] = 0.0;
      input.target_acceleration[j] = 0.0;
    }

    ruckig::Result result = otg.update(input, output);
    int guard = 0;
    while (result == ruckig::Result::Working && guard++ < 10000) {
      t_abs += 0.01;
      std::vector<double> positions(dof);
      for (std::size_t j = 0; j < dof; ++j) {
        positions[j] = output.new_position[j];
      }
      core::JointState wp;
      SetJointState(&wp, names, positions);
      AddTrajectoryPoint(&smoothed, wp, t_abs);
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
