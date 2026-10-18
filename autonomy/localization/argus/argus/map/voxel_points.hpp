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
 * @file voxel_points.hpp
 * @brief Per-voxel bucket of visual landmarks.
 */

#ifndef ARGUS_MAP_VOXEL_POINTS_HPP_
#define ARGUS_MAP_VOXEL_POINTS_HPP_

#include <vector>

#include "argus/map/visual_point.hpp"

namespace argus {
namespace map {

/** @brief Per-voxel bucket of visual points (owns pointed VisualPoint*). */
class VoxelPoints
{
public:
    std::vector<VisualPoint*>
        voxel_points; /**< @brief Landmarks in this voxel. */
    int count = 0;    /**< @brief Observation / bookkeeping count. */

    /** @param num Initial count field (default 0). */
    explicit VoxelPoints(int num = 0) : count(num) {}
    ~VoxelPoints();

    VoxelPoints(const VoxelPoints&) = delete;
    VoxelPoints& operator=(const VoxelPoints&) = delete;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_VOXEL_POINTS_HPP_
