/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/core/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace core {

FlatTrajectory ToFlat(const RobotTrajectory& traj) {
  FlatTrajectory flat;
  if (traj.waypoints.empty()) {
    return flat;
  }
  flat.joint_names = traj.waypoints.front().names;
  flat.time_from_start = traj.time_from_start;
  flat.positions.reserve(traj.waypoints.size());
  for (const auto& wp : traj.waypoints) {
    flat.positions.push_back(wp.positions);
    if (flat.joint_names.empty() && !wp.names.empty()) {
      flat.joint_names = wp.names;
    }
  }
  if (flat.time_from_start.size() != flat.positions.size()) {
    flat.time_from_start.resize(flat.positions.size(), 0.0);
    for (std::size_t i = 0; i < flat.time_from_start.size(); ++i) {
      flat.time_from_start[i] = static_cast<double>(i) * 0.1;
    }
  }
  return flat;
}

RobotTrajectory FromFlat(const FlatTrajectory& flat) {
  RobotTrajectory traj;
  traj.time_from_start = flat.time_from_start;
  traj.waypoints.reserve(flat.positions.size());
  for (const auto& pos : flat.positions) {
    JointState js;
    js.names = flat.joint_names;
    js.positions = pos;
    traj.waypoints.push_back(std::move(js));
  }
  return traj;
}

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
