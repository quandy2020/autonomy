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
 * @file voxel_key_hash.hpp
 * @brief Integer voxel coordinate used by the lidar plane map.
 */

#ifndef ATLAS_MAP_VOXEL_KEY_HASH_HPP_
#define ATLAS_MAP_VOXEL_KEY_HASH_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>

namespace atlas {
namespace map {
namespace voxel {

struct VoxelKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  VoxelKey() = default;
  VoxelKey(std::int64_t vx, std::int64_t vy, std::int64_t vz)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const {
    constexpr std::int64_t kPrime = 116101;
    constexpr std::int64_t kMaxN = 10000000000LL;
    return static_cast<std::size_t>(
        (((std::hash<std::int64_t>()(key.z) * kPrime) % kMaxN +
          std::hash<std::int64_t>()(key.y)) *
         kPrime) %
            kMaxN +
        std::hash<std::int64_t>()(key.x));
  }
};

}  // namespace voxel
}  // namespace map
}  // namespace atlas

namespace std {

template <>
struct hash<atlas::map::voxel::VoxelKey> {
  std::size_t operator()(const atlas::map::voxel::VoxelKey& key) const {
    return atlas::map::voxel::VoxelKeyHash{}(key);
  }
};

}  // namespace std

#endif  // ATLAS_MAP_VOXEL_KEY_HASH_HPP_
