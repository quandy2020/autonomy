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
 * @file occupancy_grid_data.hpp
 * @brief Cell payload for 2.5D occupancy subgrids (atlas g2p5 GridData).
 */

#ifndef ARGUS_MAP_OCCUPANCY_GRID_DATA_HPP_
#define ARGUS_MAP_OCCUPANCY_GRID_DATA_HPP_

namespace argus {
namespace map {

/** @brief Hit / visit counters and clearance height for one fine cell. */
struct OccupancyGridCell {
    /** @brief Construct with optional counters. */
    explicit OccupancyGridCell(unsigned int hit_cnt = 0,
                               unsigned int visit_cnt = 0)
        : hit_cnt(hit_cnt), visit_cnt(visit_cnt) {}

    unsigned int hit_cnt = 0;   /**< @brief Occupied (hit) observations. */
    unsigned int visit_cnt = 0; /**< @brief Total observations (hit + miss). */
    /** @brief Lowest relative height seen (m); large default = unset. */
    float height = 10000.f;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_OCCUPANCY_GRID_DATA_HPP_
