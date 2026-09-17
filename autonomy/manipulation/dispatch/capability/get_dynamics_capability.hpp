/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <vector>

#include "autonomy/manipulation/dispatch/capability/capability.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

/**
 * @brief Gravity / Coriolis torques (MoveIt dynamics_solver lite).
 */
class GetDynamicsCapability : public Capability {
 public:
  std::string Name() const override { return "get_dynamics"; }
  bool Init(ManipulationServer* server) override;

  /** @brief true when Pinocchio FEATURE is linked. */
  bool HasPinocchio() const;

  /**
   * @brief Gravity compensation torques at @p positions.
   * @return true if @p torques filled.
   */
  bool GetGravityTorques(const std::vector<double>& positions,
                         std::vector<double>* torques) const;

  /**
   * @brief Nonlinear effects (Coriolis + gravity) at state.
   */
  bool GetCoriolisTorques(const std::vector<double>& positions,
                          const std::vector<double>& velocities,
                          std::vector<double>* torques) const;

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
