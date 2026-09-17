/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/execution/logging_trajectory_controller.hpp"

#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {

bool LoggingTrajectoryController::Init(const std::string& controller_id) {
  controller_id_ = controller_id;
  AINFO << "LoggingTrajectoryController id=" << controller_id_;
  return true;
}

bool LoggingTrajectoryController::FollowJointTrajectory(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& joint_trajectory) {
  active_ = true;
  int n_effort = 0;
  double effort_l1 = 0.0;
  for (int i = 0; i < joint_trajectory.points_size(); ++i) {
    const auto& pt = joint_trajectory.points(i);
    if (pt.effort_size() == 0) {
      continue;
    }
    ++n_effort;
    for (double e : pt.effort()) {
      effort_l1 += std::abs(e);
    }
  }
  AINFO << "LoggingTrajectoryController execute waypoints="
        << joint_trajectory.points_size() << " with_effort=" << n_effort
        << " effort_l1=" << effort_l1 << " controller=" << controller_id_;
  active_ = false;
  return joint_trajectory.points_size() > 0;
}

void LoggingTrajectoryController::Cancel() {
  active_ = false;
}

bool LoggingTrajectoryController::IsActive() const {
  return active_;
}


AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(LoggingTrajectoryController, common::ControllerInterface);

}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
