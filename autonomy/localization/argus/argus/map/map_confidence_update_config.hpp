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
 * @file map_confidence_update_config.hpp
 * @brief Tunable gains for @ref BumpMapConfidence.
 */

#ifndef ARGUS_MAP_MAP_CONFIDENCE_UPDATE_CONFIG_HPP_
#define ARGUS_MAP_MAP_CONFIDENCE_UPDATE_CONFIG_HPP_

namespace argus {
namespace map {

/** @brief Tunable gains for @ref BumpMapConfidence. */
struct MapConfidenceUpdateConfig {
    float good_obs_gain = 0.05f; /**< @brief Additive gain on good updates. */
    float bad_obs_loss = 0.08f;  /**< @brief Subtractive loss on bad updates. */
    float degeneracy_penalty =
        0.5f; /**< @brief Multiplier after degenerate update. */
    float confidence_min = 0.f; /**< @brief Lower clamp. */
    float confidence_max = 1.f; /**< @brief Upper clamp. */
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_MAP_CONFIDENCE_UPDATE_CONFIG_HPP_
