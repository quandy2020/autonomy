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
 * @file voxel_location.hpp
 * @brief Integer voxel hash key for Argus LIO maps.
 */

#ifndef ARGUS_MAP_VOXEL_LOCATION_HPP_
#define ARGUS_MAP_VOXEL_LOCATION_HPP_

#include <cstdint>
#include <functional>

namespace argus {
namespace map {

constexpr int64_t kVoxelMapHashP = 116101;
constexpr int64_t kVoxelMapMaxN = 10000000000LL;

/** @brief Discrete (x,y,z) index of a root voxel cell. */
class VoxelLocation
{
public:
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;

    VoxelLocation(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
        : x(vx), y(vy), z(vz) {}

    bool operator==(const VoxelLocation& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

}  // namespace map
}  // namespace argus

namespace std {
template <>
struct hash<argus::map::VoxelLocation> {
    int64_t operator()(const argus::map::VoxelLocation& s) const {
        return ((((s.z) * argus::map::kVoxelMapHashP) %
                     argus::map::kVoxelMapMaxN +
                 (s.y)) *
                argus::map::kVoxelMapHashP) %
                   argus::map::kVoxelMapMaxN +
               (s.x);
    }
};
}  // namespace std

#endif  // ARGUS_MAP_VOXEL_LOCATION_HPP_
