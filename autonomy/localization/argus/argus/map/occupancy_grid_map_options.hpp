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
 * @file occupancy_grid_map_options.hpp
 * @brief Options for OccupancyGridMap.
 */

#ifndef ARGUS_MAP_OCCUPANCY_GRID_MAP_OPTIONS_HPP_
#define ARGUS_MAP_OCCUPANCY_GRID_MAP_OPTIONS_HPP_

namespace argus {
namespace map {

/** @brief Resolution and occupancy threshold for OccupancyGridMap. */
struct OccupancyGridMapOptions {
    float resolution = 0.1f;      /**< @brief Fine cell size (m). */
    float occupancy_ratio = 0.3f; /**< @brief hit/visit threshold for occupied. */
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_OCCUPANCY_GRID_MAP_OPTIONS_HPP_
