/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/pipeline/densify_joint_trajectory_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/model/joint_state_utilities.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

bool DensifyJointTrajectoryAdapter::Adapt(MotionPlanRequest* request,
                               ::autonomy::manipulation::proto::MotionPlanResponse* response) const {
  if (!request || !response || !response->success() ||
      response->trajectory().points_size() < 2) {
    return true;
  }
  automsgs::msgs::trajectory_msgs::JointTrajectory dense;
  for (int i = 0; i + 1 < response->trajectory().points_size(); ++i) {
    const automsgs::msgs::sensor_msgs::JointState a =
        MakeJointStateFromPoint(response->trajectory(), i);
    const automsgs::msgs::sensor_msgs::JointState b =
        MakeJointStateFromPoint(response->trajectory(), i + 1);
    AddTrajectoryPoint(&dense, a, 0.0);
    double max_dq = 0.0;
    const int n = std::min(a.position_size(), b.position_size());
    for (int j = 0; j < n; ++j) {
      max_dq = std::max(max_dq, std::abs(b.position(j) - a.position(j)));
    }
    const int mid = static_cast<int>(std::ceil(max_dq / max_joint_step_)) - 1;
    for (int k = 1; k <= mid; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(mid + 1);
      automsgs::msgs::sensor_msgs::JointState wp = a;
      ResizeJointState(&wp, n);
      for (int j = 0; j < n; ++j) {
        wp.set_position(j, a.position(j) + t * (b.position(j) - a.position(j)));
      }
      AddTrajectoryPoint(&dense, wp, 0.0);
    }
  }
  AddTrajectoryPoint(
      &dense,
      MakeJointStateFromPoint(response->trajectory(),
                              response->trajectory().points_size() - 1),
      0.0);
  for (int i = 0; i < dense.points_size(); ++i) {
    SetDurationSeconds(0.05 * static_cast<double>(i),
                       dense.mutable_points(i)->mutable_time_from_start());
  }
  *response->mutable_trajectory() = std::move(dense);
  return true;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(DensifyJointTrajectoryAdapter,
                                        PlanningRequestAdapter);

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
