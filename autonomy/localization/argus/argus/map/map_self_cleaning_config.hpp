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
 * @file map_self_cleaning_config.hpp
 * @brief Tunables for @ref MapSelfCleaning.
 */

#ifndef ARGUS_MAP_MAP_SELF_CLEANING_CONFIG_HPP_
#define ARGUS_MAP_MAP_SELF_CLEANING_CONFIG_HPP_

#include <cstdint>

#include "argus/map/map_confidence_update_config.hpp"

namespace argus {
namespace map {

/** @brief Enable / thresholds / interval for map self-cleaning. */
struct MapSelfCleaningConfig {
    bool enable = false;
    float confidence_threshold = 0.3f;
    float degeneracy_penalty = 0.5f;
    uint32_t min_observation_count_to_keep = 3;
    double interval_sec = 10.0;
    MapConfidenceUpdateConfig confidence_update;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_MAP_SELF_CLEANING_CONFIG_HPP_
