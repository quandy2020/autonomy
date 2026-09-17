/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/dispatch/capability/fk_ik_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/manipulation_server.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

bool FkIkCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

bool FkIkCapability::ComputeFk(
    const automsgs::msgs::sensor_msgs::JointState& joints,
    automsgs::msgs::geometry_msgs::Pose* pose) const {
  return server_ && server_->kinematics() &&
         server_->kinematics()->GetPositionFK(joints, pose);
}

ErrorCode FkIkCapability::ComputeIk(
    const automsgs::msgs::geometry_msgs::Pose& pose,
    const automsgs::msgs::sensor_msgs::JointState& seed,
    automsgs::msgs::sensor_msgs::JointState* solution) const {
  if (!server_ || !server_->kinematics()) {
    return ErrorCode::NO_INVERSE_KINEMATICS_SOLUTION;
  }
  return server_->kinematics()->GetPositionIK(pose, seed, {}, solution);
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(FkIkCapability, Capability);

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
