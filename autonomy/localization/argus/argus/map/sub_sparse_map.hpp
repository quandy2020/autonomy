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
 * @file sub_sparse_map.hpp
 * @brief Working set retrieved for one visual ESIKF update.
 */

#ifndef ARGUS_MAP_SUB_SPARSE_MAP_HPP_
#define ARGUS_MAP_SUB_SPARSE_MAP_HPP_

#include <vector>

#include "argus/frontend/types.hpp"
#include "argus/map/visual_point.hpp"

namespace argus {
namespace map {

/**
 * @brief Working set retrieved for one visual update (SubSparseMap).
 *
 * Owned by VIO; types live with the map module for shared layout.
 */
struct SubSparseMap {
    std::vector<float>
        propa_errors;          /**< @brief Propagation reprojection errors. */
    std::vector<float> errors; /**< @brief Update reprojection errors. */
    std::vector<std::vector<float>>
        warp_patch;                 /**< @brief Warped patch intensities. */
    std::vector<int> search_levels; /**< @brief Pyramid levels for matching. */
    std::vector<VisualPoint*> voxel_points; /**< @brief Matched map points. */
    std::vector<double> inv_expo_list; /**< @brief Inverse exposure times. */
    std::vector<frontend::PointWithVar>
        add_from_voxel_map; /**< @brief New points from LiDAR map. */

    SubSparseMap();
    /** @brief Clear all working vectors for the next visual update. */
    void Reset();
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_SUB_SPARSE_MAP_HPP_
