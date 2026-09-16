/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/execution/simple_controller_manager.hpp"

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
  AINFO << "SimpleControllerManager execute waypoints="
        << trajectory.waypoints.size()
        << " controller=" << controller_id_;
  active_ = false;
  return !trajectory.waypoints.empty();
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
