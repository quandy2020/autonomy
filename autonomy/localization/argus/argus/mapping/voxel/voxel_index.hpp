/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_MAPPING_VOXEL_VOXEL_INDEX_HPP_
#define ARGUS_MAPPING_VOXEL_VOXEL_INDEX_HPP_

#include <cmath>
#include <cstdint>
#include <functional>

#include <Eigen/Core>

#include "argus/mapping/voxel/voxel_key.hpp"

namespace argus {
namespace map {

/**
 * @brief Discrete voxel coordinate helpers over `vxslam::VoxelKey`.
 *
 * Resolution is metres per cell. Indexing uses floor division so that
 * world points map stably into integer grid cells.
 */
class VoxelIndex {
public:
    VoxelIndex() = default;
    explicit VoxelIndex(double resolution_m) : resolution_(resolution_m) {}

    double resolution() const { return resolution_; }
    void set_resolution(double resolution_m) { resolution_ = resolution_m; }

    /** @brief World point → voxel key. */
    vxslam::VoxelKey KeyOf(const Eigen::Vector3d& p) const {
        const double inv = 1.0 / resolution_;
        return vxslam::VoxelKey(static_cast<std::int64_t>(std::floor(p.x() * inv)),
                                static_cast<std::int64_t>(std::floor(p.y() * inv)),
                                static_cast<std::int64_t>(std::floor(p.z() * inv)));
    }

    /** @brief Voxel key → world-frame cell origin (min corner). */
    Eigen::Vector3d OriginOf(const vxslam::VoxelKey& key) const {
        return Eigen::Vector3d(static_cast<double>(key.x) * resolution_,
                               static_cast<double>(key.y) * resolution_,
                               static_cast<double>(key.z) * resolution_);
    }

    /** @brief Voxel key → world-frame cell centre. */
    Eigen::Vector3d CenterOf(const vxslam::VoxelKey& key) const {
        const double h = 0.5 * resolution_;
        return OriginOf(key) + Eigen::Vector3d(h, h, h);
    }

private:
    double resolution_ = 0.5;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAPPING_VOXEL_VOXEL_INDEX_HPP_
