/*
 * Copyright 2026 The Openbot Authors
 *
 * Optional urdfdom-backed joint load (FEATURE urdfdom). Falls back to regex
 * parser when AUTONOMY_HAS_URDFDOM is undefined.
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/manipulation/model/urdf_joints.hpp"

namespace autonomy {
namespace manipulation {
namespace core {

/**
 * @brief Load movable joints preferring urdfdom when compiled with FEATURE.
 *
 * Always falls back to @ref LoadUrdfJoints on failure / missing FEATURE so
 * production builds remain usable without urdfdom.
 */
bool LoadUrdfJointsPreferred(const std::string& path,
                             std::vector<UrdfJointInfo>* joints,
                             std::string* error = nullptr);

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
