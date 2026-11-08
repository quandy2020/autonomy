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
 * @file voxel_grid.hpp
 * @brief SoA point storage with a voxel hash index.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace navigation
{
namespace memory
{

/** @brief Sparse point set indexed by cubic voxels. */
class VoxelGrid
{
public:
    /** @brief Callback invoked for every point inside the query radius. */
    using Visitor = std::function<void(double x, double y, double z, std::uint32_t id)>;

    /**
     * @brief Construct with a cubic voxel resolution.
     *
     * @param[in] resolution Voxel edge length in metres (> 0).
     */
    explicit VoxelGrid(double resolution = 0.1);

    /**
     * @brief Insert one point.
     *
     * @param[in] x X coordinate in metres.
     * @param[in] y Y coordinate in metres.
     * @param[in] z Z coordinate in metres.
     * @param[in] id Caller payload returned by queries.
     */
    void Insert(double x, double y, double z, std::uint32_t id);

    /**
     * @brief Visit every point within a sphere.
     *
     * @param[in] x Query centre x in metres.
     * @param[in] y Query centre y in metres.
     * @param[in] z Query centre z in metres.
     * @param[in] radius Query radius in metres.
     * @param[in] visitor Callback; must not mutate the grid.
     */
    void QueryRadius(double x, double y, double z, double radius,
                     const Visitor& visitor) const;

    /** @brief Number of stored points. */
    [[nodiscard]] std::size_t Size() const;

    /** @brief Remove all points and cells. */
    void Clear();

    /** @brief Voxel edge length in metres. */
    [[nodiscard]] double Resolution() const;

private:
    /** @brief Pack a voxel coordinate into a 64-bit key. */
    static std::uint64_t Encode(std::int32_t x, std::int32_t y, std::int32_t z);

    /** @brief Voxel edge length in metres. */
    double resolution_{0.1};
    /** @brief SoA x coordinates in metres. */
    std::vector<double> x_;
    /** @brief SoA y coordinates in metres. */
    std::vector<double> y_;
    /** @brief SoA z coordinates in metres. */
    std::vector<double> z_;
    /** @brief Caller payload per point. */
    std::vector<std::uint32_t> ids_;
    /** @brief Voxel key -> point indices. */
    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> cells_;
};

}  // namespace memory
}  // namespace navigation
