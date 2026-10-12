/*
 * Copyright 2026 The Openbot Authors
 */

#include "automanip/dispatch/capability/validate_trajectory_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "automanip/constraints/constraint_samplers.hpp"
#include "automanip/manipulation_server.hpp"

namespace automanip {
namespace dispatch {

bool ValidateTrajectoryCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

ErrorCode ValidateTrajectoryCapability::Validate(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory,
    const planner::MotionPlanRequest& constraints) const {
  if (!server_ || !server_->scene()) {
    return ErrorCode::INVALID_PLANNING_SCENE;
  }
  if (!server_->scene()->IsPathValid(trajectory)) {
    return ErrorCode::INVALID_MOTION_PLAN;
  }
  planner::MotionPlanRequest req = constraints;
  req.kinematics = server_->SharedKinematics();
  if (!constraints::SatisfiesPathConstraints(req, trajectory)) {
    return ErrorCode::INVALID_MOTION_PLAN;
  }
  return ErrorCode::SUCCESS;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ValidateTrajectoryCapability,
                                        Capability);

}  // namespace dispatch
}  // namespace automanip
