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

#include "navigation/memory/spatial_index.hpp"

namespace navigation
{
namespace memory
{

SpatialIndex::SpatialIndex(double resolution) : grid_(resolution)
{
}

void SpatialIndex::Insert(std::uint64_t id, const common::Pose2D& pose)
{
    grid_.Insert(pose.x(), pose.y(), 0.0, static_cast<std::uint32_t>(id));
}

std::vector<std::uint64_t> SpatialIndex::QueryRadius(const common::Pose2D& pose,
                                                     double radius) const
{
    std::vector<std::uint64_t> ids;
    grid_.QueryRadius(pose.x(), pose.y(), 0.0, radius,
                      [&ids](double, double, double, std::uint32_t id) {
                          ids.push_back(id);
                      });
    return ids;
}

std::size_t SpatialIndex::Size() const
{
    return grid_.Size();
}

void SpatialIndex::Clear()
{
    grid_.Clear();
}

}  // namespace memory
}  // namespace navigation
