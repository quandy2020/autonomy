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

#include "navigation/memory/voxel_grid.hpp"

#include <algorithm>
#include <cmath>

namespace navigation
{
namespace memory
{

VoxelGrid::VoxelGrid(double resolution)
    : resolution_(std::isfinite(resolution) && resolution > 0.0 ? resolution : 0.1)
{
}

std::uint64_t VoxelGrid::Encode(std::int32_t x, std::int32_t y, std::int32_t z)
{
    constexpr std::uint64_t kMask = (1ULL << 21) - 1ULL;
    return (static_cast<std::uint64_t>(x) & kMask) |
           ((static_cast<std::uint64_t>(y) & kMask) << 21) |
           ((static_cast<std::uint64_t>(z) & kMask) << 42);
}

void VoxelGrid::Insert(double x, double y, double z, std::uint32_t id)
{
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
        return;
    }
    const std::int32_t vx = static_cast<std::int32_t>(std::floor(x / resolution_));
    const std::int32_t vy = static_cast<std::int32_t>(std::floor(y / resolution_));
    const std::int32_t vz = static_cast<std::int32_t>(std::floor(z / resolution_));
    const std::uint64_t key = Encode(vx, vy, vz);
    cells_[key].push_back(static_cast<std::uint32_t>(x_.size()));
    x_.push_back(x);
    y_.push_back(y);
    z_.push_back(z);
    ids_.push_back(id);
}

void VoxelGrid::QueryRadius(double x, double y, double z, double radius,
                            const Visitor& visitor) const
{
    if (!visitor || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(radius) || radius < 0.0)
    {
        return;
    }
    const std::int32_t min_x =
        static_cast<std::int32_t>(std::floor((x - radius) / resolution_));
    const std::int32_t max_x =
        static_cast<std::int32_t>(std::floor((x + radius) / resolution_));
    const std::int32_t min_y =
        static_cast<std::int32_t>(std::floor((y - radius) / resolution_));
    const std::int32_t max_y =
        static_cast<std::int32_t>(std::floor((y + radius) / resolution_));
    const std::int32_t min_z =
        static_cast<std::int32_t>(std::floor((z - radius) / resolution_));
    const std::int32_t max_z =
        static_cast<std::int32_t>(std::floor((z + radius) / resolution_));
    const double radius_squared = radius * radius;
    for (std::int32_t vx = min_x; vx <= max_x; ++vx)
    {
        for (std::int32_t vy = min_y; vy <= max_y; ++vy)
        {
            for (std::int32_t vz = min_z; vz <= max_z; ++vz)
            {
                const auto iterator = cells_.find(Encode(vx, vy, vz));
                if (iterator == cells_.end())
                {
                    continue;
                }
                for (std::uint32_t index : iterator->second)
                {
                    const double dx = x_[index] - x;
                    const double dy = y_[index] - y;
                    const double dz = z_[index] - z;
                    if (dx * dx + dy * dy + dz * dz <= radius_squared)
                    {
                        visitor(x_[index], y_[index], z_[index], ids_[index]);
                    }
                }
            }
        }
    }
}

std::size_t VoxelGrid::Size() const
{
    return x_.size();
}

void VoxelGrid::Clear()
{
    x_.clear();
    y_.clear();
    z_.clear();
    ids_.clear();
    cells_.clear();
}

double VoxelGrid::Resolution() const
{
    return resolution_;
}

}  // namespace memory
}  // namespace navigation
