/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planning/time_parameterization.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace manipulation {
namespace trajectory {

bool ApplyTotg(core::RobotTrajectory* traj, const TimeParamOptions& options) {
  if (!traj || traj->waypoints.size() < 2) {
    return false;
  }
  const double vmax = std::max(1e-6, options.max_velocity);
  traj->time_from_start.assign(traj->waypoints.size(), 0.0);
  double t = 0.0;
  for (std::size_t i = 1; i < traj->waypoints.size(); ++i) {
    const auto& a = traj->waypoints[i - 1].positions;
    const auto& b = traj->waypoints[i].positions;
    const std::size_t n = std::min(a.size(), b.size());
    double max_dq = 0.0;
    for (std::size_t j = 0; j < n; ++j) {
      max_dq = std::max(max_dq, std::abs(b[j] - a[j]));
    }
    t += max_dq / vmax;
    traj->time_from_start[i] = t;
  }
  return true;
}

}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
