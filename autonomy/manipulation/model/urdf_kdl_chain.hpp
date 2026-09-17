/*
 * Copyright 2026 The Openbot Authors
 *
 * Build a KDL::Chain from a URDF file (lightweight, no urdfdom/kdl_parser).
 */

#pragma once

#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

namespace autonomy {
namespace manipulation {
namespace model {

/**
 * @brief KDL chain plus joint names and position bounds for inverse / forward
 *        kinematics solvers.
 */
struct KdlChainDescription {
  KDL::Chain chain;
  std::vector<std::string> joint_names;
  KDL::JntArray position_lower_bounds;
  KDL::JntArray position_upper_bounds;
};

/**
 * @brief Extract a KDL chain base→tip and movable joint limits from URDF.
 * @param[in] urdf_path Path to URDF file.
 * @param[in] base_link Base link of the chain.
 * @param[in] tip_link Tip / end-effector link.
 * @param[out] chain_description Populated chain model (must be non-null).
 * @param[out] error Optional human-readable failure reason.
 * @return true on successful extraction.
 */
bool BuildKdlChainFromUrdfFile(const std::string& urdf_path,
                               const std::string& base_link,
                               const std::string& tip_link,
                               KdlChainDescription* chain_description,
                               std::string* error = nullptr);

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
