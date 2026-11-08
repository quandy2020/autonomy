/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_MAPPING_VOXEL_VOXEL_KEY_HPP_
#define ARGUS_MAPPING_VOXEL_VOXEL_KEY_HPP_

#include <cstdint>
#include <functional>

namespace argus {
namespace vxslam {

/** @brief Integer hash key for a voxel cell. */
struct VoxelKey {
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;

    VoxelKey() = default;
    VoxelKey(int64_t vx, int64_t vy, int64_t vz) : x(vx), y(vy), z(vz) {}

    bool operator==(const VoxelKey& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

}  // namespace vxslam
}  // namespace argus

namespace std {
template <>
struct hash<argus::vxslam::VoxelKey> {
    size_t operator()(const argus::vxslam::VoxelKey& s) const {
        constexpr int64_t kP = 116101;
        constexpr int64_t kMaxN = 10000000000LL;
        return static_cast<size_t>(
            (((hash<int64_t>()(s.z) * kP) % kMaxN + hash<int64_t>()(s.y)) * kP) %
                kMaxN +
            hash<int64_t>()(s.x));
    }
};
}  // namespace std

#endif  // ARGUS_MAPPING_VOXEL_VOXEL_KEY_HPP_
