/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/model/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace core {

FlatTrajectory ToFlat(const RobotTrajectory& traj) {
  FlatTrajectory flat;
  for (const auto& n : traj.joint_names()) {
    flat.joint_names.push_back(n);
  }
  flat.positions.reserve(static_cast<std::size_t>(traj.points_size()));
  flat.time_from_start.reserve(static_cast<std::size_t>(traj.points_size()));
  for (int i = 0; i < traj.points_size(); ++i) {
    const auto& pt = traj.points(i);
    std::vector<double> pos(pt.positions().begin(), pt.positions().end());
    flat.positions.push_back(std::move(pos));
    flat.time_from_start.push_back(GetTrajectoryTime(traj, i));
  }
  return flat;
}

RobotTrajectory FromFlat(const FlatTrajectory& flat) {
  RobotTrajectory traj;
  for (const auto& n : flat.joint_names) {
    traj.add_joint_names(n);
  }
  for (std::size_t i = 0; i < flat.positions.size(); ++i) {
    auto* pt = traj.add_points();
    for (double q : flat.positions[i]) {
      pt->add_positions(q);
    }
    double t = 0.0;
    if (i < flat.time_from_start.size()) {
      t = flat.time_from_start[i];
    } else {
      t = static_cast<double>(i) * 0.1;
    }
    SetDurationSeconds(t, pt->mutable_time_from_start());
  }
  return traj;
}

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
