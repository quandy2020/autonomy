/*
 * Copyright 2026 The Openbot Authors
 *
 * Lightweight SRDF planning-group and collision-pair extraction.
 */

#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autonomy/manipulation/model/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace model {

/**
 * @brief Parse SRDF `<group>` blocks into @ref JointModelGroup entries.
 * @param[in] srdf_path Path to SRDF file.
 * @param[out] groups Map from group name to joint/link meta (must be non-null).
 * @param[out] error Optional human-readable failure reason.
 * @return true on successful parse (empty groups allowed).
 */
bool LoadPlanningGroupsFromSrdf(
    const std::string& srdf_path,
    std::unordered_map<std::string, JointModelGroup>* groups,
    std::string* error = nullptr);

/**
 * @brief Parse SRDF `<disable_collisions>` pairs for ACM seeding.
 * @param[in] srdf_path Path to SRDF file.
 * @param[out] link_pairs Link-name pairs that should skip collision checks.
 * @param[out] error Optional human-readable failure reason.
 * @return true on successful parse.
 */
bool LoadDisabledCollisionsFromSrdf(
    const std::string& srdf_path,
    std::vector<std::pair<std::string, std::string>>* link_pairs,
    std::string* error = nullptr);

/**
 * @brief Parse SRDF `<end_effector>` tags into automsgs SrdfEndEffector.
 */
bool LoadEndEffectorsFromSrdf(const std::string& srdf_path,
                              std::vector<SrdfEndEffector>* end_effectors,
                              std::string* error = nullptr);

/**
 * @brief Parse SRDF `<passive_joint>` names.
 */
bool LoadPassiveJointsFromSrdf(const std::string& srdf_path,
                               std::vector<std::string>* passive_joints,
                               std::string* error = nullptr);

}  // namespace model
}  // namespace manipulation
}  // namespace autonomy
