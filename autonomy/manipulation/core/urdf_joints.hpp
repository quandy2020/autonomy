/*
 * Copyright 2026 The Openbot Authors
 *
 * Lightweight URDF joint extraction (no map/strata dependency).
 */

#pragma once

#include <string>
#include <vector>

namespace autonomy {
namespace manipulation {
namespace core {

/** @brief Parsed URDF joint limits and optional mimic metadata. */
struct UrdfJointInfo {
  std::string name;
  std::string type;  // revolute / continuous / prismatic / fixed / …
  bool has_position_limits = false;
  double lower = -3.141592653589793;
  double upper = 3.141592653589793;
  double velocity = 1.0;
  double effort = 0.0;
  std::string mimic_joint;
  double mimic_multiplier = 1.0;
  double mimic_offset = 0.0;
};

/**
 * @brief Load movable joints (revolute/continuous/prismatic) from a URDF file.
 * @param[in] path Path to URDF file.
 * @param[out] joints Output joint list (must be non-null).
 * @param[out] error Optional human-readable failure reason.
 * @return true on successful parse.
 */
bool LoadUrdfJoints(const std::string& path, std::vector<UrdfJointInfo>* joints,
                    std::string* error = nullptr);

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
