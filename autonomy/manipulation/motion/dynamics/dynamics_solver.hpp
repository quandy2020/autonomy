/*
 * Copyright 2026 The Openbot Authors
 *
 * Dynamics solver stub (MoveIt dynamics_solver lite / optional Pinocchio hook).
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autonomy/manipulation/model/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace dynamics {

/**
 * @brief Inverse-dynamics / gravity lite interface.
 *
 * Full MoveIt uses KDL ChainIdSolver / optional Pinocchio. This stub returns
 * zero torques unless a FEATURE backend is linked later.
 */
class DynamicsSolver {
 public:
  virtual ~DynamicsSolver() = default;

  virtual bool Init(const std::string& /*group*/) { return true; }

  /**
   * @brief Gravity compensation torques at @p positions (Nm).
   * @return true if @p torques filled (may be zeros in stub).
   */
  virtual bool GetGravityTorques(const std::vector<double>& positions,
                                 std::vector<double>* torques) const {
    if (!torques) {
      return false;
    }
    torques->assign(positions.size(), 0.0);
    return true;
  }

  /**
   * @brief Coriolis + gravity (stub: zeros).
   */
  virtual bool GetCoriolisTorques(const std::vector<double>& positions,
                                  const std::vector<double>& /*velocities*/,
                                  std::vector<double>* torques) const {
    return GetGravityTorques(positions, torques);
  }
};

}  // namespace dynamics
}  // namespace manipulation
}  // namespace autonomy
