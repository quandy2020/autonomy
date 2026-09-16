/*
 * Copyright 2026 The Openbot Authors
 *
 * Lightweight SRDF group + disable_collisions extraction.
 */

#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autonomy/manipulation/core/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace core {

/**
 * @brief Parse SRDF `<group>` blocks into @ref JointModelGroup entries.
 * @param[in] path Path to SRDF file.
 * @param[out] groups Map from group name to joint/link meta (must be non-null).
 * @param[out] error Optional human-readable failure reason.
 * @return true on successful parse (empty groups allowed).
 */
bool LoadSrdfGroups(const std::string& path,
                    std::unordered_map<std::string, JointModelGroup>* groups,
                    std::string* error = nullptr);

/**
 * @brief Parse SRDF `<disable_collisions>` pairs for ACM seeding.
 * @param[in] path Path to SRDF file.
 * @param[out] pairs Link-name pairs that should skip collision checks.
 * @param[out] error Optional human-readable failure reason.
 * @return true on successful parse.
 */
bool LoadSrdfDisableCollisions(
    const std::string& path,
    std::vector<std::pair<std::string, std::string>>* pairs,
    std::string* error = nullptr);

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
