/*
 * Copyright 2026 The Openbot Authors
 *
 * Optional Ruckig trajectory smoother (FEATURES ruckig).
 */

#include "autonomy/manipulation/pipeline/time_parameterization.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/model/joint_state_utilities.hpp"

#ifdef AUTONOMY_HAS_RUCKIG
#include <ruckig/ruckig.hpp>
#endif

namespace autonomy {
namespace manipulation {
namespace trajectory {

bool ApplyRuckigTrajectorySmoothing(automsgs::msgs::trajectory_msgs::JointTrajectory* trajectory, const TimeParameterizationOptions& options) {
  if (!trajectory || trajectory->points_size() < 2) {
    return false;
  }

#ifdef AUTONOMY_HAS_RUCKIG
  const std::size_t dof =
      static_cast<std::size_t>(trajectory->points(0).positions_size());
  if (dof == 0) {
    return false;
  }

  // Piecewise online Ruckig between consecutive waypoints.
  ruckig::Ruckig<ruckig::DynamicDOFs> online_trajectory_generator(static_cast<int>(dof), 0.01);
  ruckig::InputParameter<ruckig::DynamicDOFs> input(static_cast<int>(dof));
  ruckig::OutputParameter<ruckig::DynamicDOFs> output(static_cast<int>(dof));

  const double max_velocity = std::max(1e-3, options.max_velocity());
  const double max_acceleration = std::max(1e-3, options.max_acceleration());
  for (std::size_t j = 0; j < dof; ++j) {
    input.max_velocity()[j] = max_velocity;
    input.max_acceleration()[j] = max_acceleration;
    input.max_jerk[j] = max_acceleration * 10.0;
  }

  automsgs::msgs::trajectory_msgs::JointTrajectory smoothed;
  AddTrajectoryPoint(&smoothed, MakeJointStateFromPoint(*trajectory, 0), 0.0);
  double t_abs = 0.0;

  std::vector<std::string> names(trajectory->joint_names().begin(),
                                 trajectory->joint_names().end());

  for (int i = 0; i + 1 < trajectory->points_size(); ++i) {
    const auto& a = trajectory->points(i);
    const auto& b = trajectory->points(i + 1);
    for (std::size_t j = 0; j < dof; ++j) {
      input.current_position[j] = a.positions(static_cast<int>(j));
      input.current_velocity[j] = 0.0;
      input.current_acceleration[j] = 0.0;
      input.target_position[j] = b.positions(static_cast<int>(j));
      input.target_velocity[j] = 0.0;
      input.target_acceleration[j] = 0.0;
    }

    ruckig::Result result = online_trajectory_generator.update(input, output);
    int guard = 0;
    while (result == ruckig::Result::Working && guard++ < 10000) {
      t_abs += 0.01;
      std::vector<double> positions(dof);
      for (std::size_t j = 0; j < dof; ++j) {
        positions[j] = output.new_position[j];
      }
      automsgs::msgs::sensor_msgs::JointState wp;
      SetJointState(&wp, names, positions);
      AddTrajectoryPoint(&smoothed, wp, t_abs);
      output.pass_to_input(input);
      result = online_trajectory_generator.update(input, output);
    }
    if (result != ruckig::Result::Finished &&
        result != ruckig::Result::Working) {
      AWARN << "ApplyRuckigTrajectorySmoothing: segment failed, falling back to time-optimal trajectory generation";
      return ApplyTimeOptimalTrajectoryGeneration(trajectory, options);
    }
  }

  *trajectory = std::move(smoothed);
  return true;
#else
  (void)options;
  AWARN << "ApplyRuckigTrajectorySmoothing: AUTONOMY_HAS_RUCKIG off; no-op (use ApplyTimeOptimalTrajectoryGeneration)";
  return true;
#endif
}

bool ApplyRuckigTrajectorySmoothing(automsgs::msgs::trajectory_msgs::JointTrajectory* trajectory) {
  TimeParameterizationOptions options;
  return ApplyRuckigTrajectorySmoothing(trajectory, options);
}

}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
