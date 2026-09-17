/*
 * Copyright 2026 The Openbot Authors
 *
 * Lightweight URDF joint extraction into automsgs JointModel.
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/manipulation/model/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace model {

/**
 * @brief Load movable joints (revolute/continuous/prismatic) from a URDF file.
 */
bool LoadJointsFromUrdfFile(const std::string& urdf_path,
                            std::vector<JointModel>* joints,
                            std::string* error = nullptr);

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
