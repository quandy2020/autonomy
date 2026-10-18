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
 * @file map_confidence.hpp
 * @brief Free helpers for per-voxel confidence (types in separate headers).
 */

#ifndef ARGUS_MAP_MAP_CONFIDENCE_HPP_
#define ARGUS_MAP_MAP_CONFIDENCE_HPP_

#include <algorithm>
#include <cstdint>

#include "argus/map/map_confidence_update_config.hpp"
#include "argus/map/map_point_confidence.hpp"

namespace argus {
namespace map {

/**
 * @brief Apply one observation to a voxel confidence record.
 * @param c Record to update (no-op if null).
 * @param under_degeneracy True if the LiDAR update was degenerate.
 * @param cfg Gain/loss parameters (default constructed).
 */
inline void BumpMapConfidence(MapPointConfidence* c, bool under_degeneracy,
                              const MapConfidenceUpdateConfig& cfg = {}) {
    if (c == nullptr) {
        return;
    }
    ++c->observation_count;
    if (under_degeneracy) {
        c->updated_under_degeneracy = true;
        c->confidence *= cfg.degeneracy_penalty;
        c->confidence -= cfg.bad_obs_loss;
    } else {
        c->confidence += cfg.good_obs_gain;
    }
    c->confidence = std::max(cfg.confidence_min,
                             std::min(cfg.confidence_max, c->confidence));
}

/**
 * @brief Whether a voxel should be pruned during self-cleaning.
 *
 * High @c observation_count anchors are retained even if confidence dips.
 * @param c Voxel confidence state.
 * @param threshold Confidence below which pruning is considered.
 * @param min_observation_count_to_keep Minimum observations to always keep.
 * @return True if the voxel is weak enough to prune.
 */
inline bool ShouldPruneVoxel(const MapPointConfidence& c, float threshold,
                             uint32_t min_observation_count_to_keep) {
    if (c.observation_count >= min_observation_count_to_keep) {
        return false;
    }
    return c.confidence < threshold;
}

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_MAP_CONFIDENCE_HPP_
