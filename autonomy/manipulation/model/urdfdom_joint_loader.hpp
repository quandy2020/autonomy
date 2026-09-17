/*
 * Copyright 2026 The Openbot Authors
 *
 * Optional urdfdom-backed joint load (FEATURE urdfdom). Falls back to the
 * regex parser when AUTONOMY_HAS_URDFDOM is undefined.
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/manipulation/model/robot_model.hpp"
#include "autonomy/manipulation/model/urdf_joint_loader.hpp"

namespace autonomy {
namespace manipulation {
namespace model {

/**
 * @brief Load movable joints preferring urdfdom when compiled with FEATURE.
 */
bool LoadJointsFromUrdfPreferUrdfdom(const std::string& urdf_path,
                                     std::vector<JointModel>* joints,
                                     std::string* error = nullptr);

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
