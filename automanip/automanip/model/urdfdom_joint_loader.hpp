/*
 * Copyright 2026 The Openbot Authors
 *
 * Optional urdfdom-backed joint load (FEATURE urdfdom). Falls back to the
 * regex parser when AUTOMANIP_HAS_URDFDOM is undefined.
 */

#pragma once

#include <string>
#include <vector>

#include "automanip/model/robot_model.hpp"
#include "automanip/model/urdf_joint_loader.hpp"

namespace automanip {
namespace model {

/**
 * @brief Load movable joints preferring urdfdom when compiled with FEATURE.
 */
bool LoadJointsFromUrdfPreferUrdfdom(const std::string& urdf_path,
                                     std::vector<JointModel>* joints,
                                     std::string* error = nullptr);

}  // namespace model
}  // namespace automanip
