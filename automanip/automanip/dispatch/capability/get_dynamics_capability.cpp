/*
 * Copyright 2026 The Openbot Authors
 */

#include "automanip/dispatch/capability/get_dynamics_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "automanip/motion/dynamics/dynamics_solver_factory.hpp"
#include "automanip/manipulation_server.hpp"

namespace automanip {
namespace dispatch {

bool GetDynamicsCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

bool GetDynamicsCapability::HasPinocchio() const {
  return dynamics::HasPinocchioDynamics();
}

bool GetDynamicsCapability::GetGravityTorques(
    const std::vector<double>& positions,
    std::vector<double>* torques) const {
  if (!server_ || !server_->dynamics_solver()) {
    return false;
  }
  return server_->dynamics_solver()->GetGravityTorques(positions, torques);
}

bool GetDynamicsCapability::GetCoriolisTorques(
    const std::vector<double>& positions,
    const std::vector<double>& velocities,
    std::vector<double>* torques) const {
  if (!server_ || !server_->dynamics_solver()) {
    return false;
  }
  return server_->dynamics_solver()->GetCoriolisTorques(positions, velocities,
                                                        torques);
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(GetDynamicsCapability, Capability);

}  // namespace dispatch
}  // namespace automanip
