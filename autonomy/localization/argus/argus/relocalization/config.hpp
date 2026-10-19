/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file config.hpp
 * @brief Top-level RelocalizationManager configuration.
 */

#ifndef ARGUS_RELOCALIZATION_CONFIG_HPP_
#define ARGUS_RELOCALIZATION_CONFIG_HPP_

#include <string>

#include "argus/relocalization/bbs/search.hpp"
#include "argus/relocalization/thread_pool.hpp"
#include "argus/relocalization/scan_context/scan_context.hpp"

namespace argus {
namespace relocalization {

/** @brief Async relocalization trigger, prior map, and search settings. */
struct RelocalizationConfig {
    RelocalizationConfig();

    bool enable = true;
    /** @brief tracking_lost | manual | cross_session_load */
    std::string trigger = "tracking_lost";
    /** @brief Consecutive lost frames before auto Request (tracking_lost). */
    int min_lost_streak = 3;
    /** @brief Minimum seconds between auto Requests. */
    double cooldown_sec = 2.0;
    /** @brief Seed BnB with current ESIKF pose when auto-triggering. */
    bool seed_with_current_pose = true;
    /**
     * @brief Prior map: `.xyz` or map package dir (`prior.xyz` / `map.xyz`).
     *        Loaded at LivoOdometry::Initialize when non-empty.
     */
    std::string prior_map_path;
    double prior_resolution = 0.5;
    RelocalizationPoolConfig thread_pool;
    BranchAndBoundConfig branch_and_bound;
    /**
     * @brief Optional Scan Context++ candidate crop before BnB.
     *        Large priors: enable to shrink search; off by default.
     */
    PlaceRecognitionConfig place_recognition;
};

}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_CONFIG_HPP_
