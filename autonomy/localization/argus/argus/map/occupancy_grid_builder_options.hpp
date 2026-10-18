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
 * @file occupancy_grid_builder_options.hpp
 * @brief Options for OccupancyGridBuilder.
 */

#ifndef ARGUS_MAP_OCCUPANCY_GRID_BUILDER_OPTIONS_HPP_
#define ARGUS_MAP_OCCUPANCY_GRID_BUILDER_OPTIONS_HPP_

#include "argus/map/occupancy_grid_map_options.hpp"

namespace argus {
namespace map {

/** @brief Floor band and range limits for OccupancyGridBuilder. */
struct OccupancyGridBuilderOptions {
    double lidar_height = 0.0;          /**< @brief Sensor z in body (m). */
    double default_floor_height = -1.0; /**< @brief Floor plane z (m). */
    double min_th_floor = 0.5;          /**< @brief Min height above floor. */
    double max_th_floor = 1.2;          /**< @brief Max height above floor. */
    float usable_scan_range = 50.f;     /**< @brief Max XY range (m). */
    OccupancyGridMapOptions map;        /**< @brief Grid resolution / ratio. */
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_OCCUPANCY_GRID_BUILDER_OPTIONS_HPP_
