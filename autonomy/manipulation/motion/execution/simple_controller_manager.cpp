/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/execution/simple_controller_manager.hpp"

#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {

bool SimpleControllerManager::Init(const std::string& controller_id) {
  controller_id_ = controller_id;
  AINFO << "SimpleControllerManager id=" << controller_id_;
  return true;
}

bool SimpleControllerManager::Execute(const core::RobotTrajectory& trajectory) {
  active_ = true;
  int n_effort = 0;
  double effort_l1 = 0.0;
  for (int i = 0; i < trajectory.points_size(); ++i) {
    const auto& pt = trajectory.points(i);
    if (pt.effort_size() == 0) {
      continue;
    }
    ++n_effort;
    for (double e : pt.effort()) {
      effort_l1 += std::abs(e);
    }
  }
  AINFO << "SimpleControllerManager execute waypoints="
        << trajectory.points_size() << " with_effort=" << n_effort
        << " effort_l1=" << effort_l1 << " controller=" << controller_id_;
  active_ = false;
  return trajectory.points_size() > 0;
}

void SimpleControllerManager::Cancel() {
  active_ = false;
}

bool SimpleControllerManager::IsActive() const {
  return active_;
}


AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(SimpleControllerManager, ControllerManager);

}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
