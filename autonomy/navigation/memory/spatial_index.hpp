/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file spatial_index.hpp
 * @brief Voxel-hashed index from pose to keyframe ids.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "navigation/common/pose2d.hpp"
#include "navigation/memory/voxel_grid.hpp"

namespace navigation
{
namespace memory
{

class SpatialIndex
{
public:
    /** @brief Construct with a voxel resolution in metres. */
    explicit SpatialIndex(double resolution = 0.1);

    /**
     * @brief Index one id at a planar pose.
     *
     * @param[in] id Keyframe id.
     * @param[in] pose Planar pose (z=0).
     */
    void Insert(std::uint64_t id, const common::Pose2D& pose);

    /**
     * @brief Return ids within a radius.
     *
     * @param[in] pose Query pose.
     * @param[in] radius Query radius in metres.
     */
    [[nodiscard]] std::vector<std::uint64_t> QueryRadius(const common::Pose2D& pose,
                                                         double radius) const;

    /** @brief Number of indexed ids. */
    [[nodiscard]] std::size_t Size() const;

    /** @brief Clear the index. */
    void Clear();

private:
    /** @brief Underlying 2D voxel storage (z=0). */
    VoxelGrid grid_;
};

}  // namespace memory
}  // namespace navigation
