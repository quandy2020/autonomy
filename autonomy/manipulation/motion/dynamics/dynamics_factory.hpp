/*
 * Copyright 2026 The Openbot Authors
 *
 * Pinocchio RNEA / gravity dynamics (FEATURE pinocchio).
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/manipulation/motion/dynamics/dynamics_solver.hpp"

namespace autonomy {
namespace manipulation {
namespace dynamics {

/**
 * @brief Create dynamics solver: Pinocchio when FEATURE, else stub.
 * @param[in] urdf_path Optional URDF for Pinocchio model build.
 */
std::shared_ptr<DynamicsSolver> CreateDynamicsSolver(
    const std::string& urdf_path = {});

bool HasPinocchioDynamics();

}  // namespace dynamics
}  // namespace manipulation
}  // namespace autonomy
