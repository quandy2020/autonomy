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
 * @file voxel_grid.hpp
 * @brief Centroid voxel-grid downsample (FAST-LIVO / former livo::VoxelGrid).
 */

#ifndef ARGUS_SENSOR_INTERNAL_VOXEL_GRID_HPP_
#define ARGUS_SENSOR_INTERNAL_VOXEL_GRID_HPP_

#include <cmath>
#include <cstdint>
#include <unordered_map>

#include "argus/sensor/point_types.hpp"

namespace argus {
namespace sensor {

/**
 * @brief Voxel-grid downsampling by per-voxel centroid (FAST-LIVO-style).
 *
 * Non-finite points are skipped; each occupied voxel outputs one point at the
 * arithmetic mean of member coordinates (other fields taken from the first
 * point).
 */
template <typename PointT>
class VoxelGrid
{
public:
    /**
     * @brief Set voxel edge lengths (clamped to a small positive minimum).
     * @param leaf_size_x Voxel size along x (m).
     * @param leaf_size_y Voxel size along y (m).
     * @param leaf_size_z Voxel size along z (m).
     */
    void SetLeafSize(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
        leaf_size_x_ = leaf_size_x > 1e-6f ? leaf_size_x : 1e-6f;
        leaf_size_y_ = leaf_size_y > 1e-6f ? leaf_size_y : 1e-6f;
        leaf_size_z_ = leaf_size_z > 1e-6f ? leaf_size_z : 1e-6f;
    }

    /**
     * @brief Set the source cloud for the next @ref Filter call.
     * @param cloud Shared input cloud.
     */
    void SetInputCloud(const typename PointCloud<PointT>::Ptr& cloud) {
        input_ = cloud;
    }

    /**
     * @brief Downsample @ref SetInputCloud into @p output (centroid per voxel).
     * @param[out] output Cleared; one point per occupied voxel, or empty if no
     * input.
     */
    void Filter(PointCloud<PointT>& output) const {
        output.clear();
        if (!input_ || input_->empty()) {
            return;
        }
        struct Accumulator {
            PointT point{};
            int count = 0;
            double sum_x = 0;
            double sum_y = 0;
            double sum_z = 0;
        };
        struct KeyHash {
            std::size_t operator()(const std::int64_t& key) const noexcept {
                return static_cast<std::size_t>(key);
            }
        };
        std::unordered_map<std::int64_t, Accumulator, KeyHash> voxels;
        voxels.reserve(input_->size() / 2 + 1);
        for (const auto& p : input_->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
                !std::isfinite(p.z)) {
                continue;
            }
            const auto ix =
                static_cast<std::int64_t>(std::floor(p.x / leaf_size_x_));
            const auto iy =
                static_cast<std::int64_t>(std::floor(p.y / leaf_size_y_));
            const auto iz =
                static_cast<std::int64_t>(std::floor(p.z / leaf_size_z_));
            const std::int64_t key = ((ix & 0x1FFFFF) << 42) |
                                     ((iy & 0x1FFFFF) << 21) | (iz & 0x1FFFFF);
            Accumulator& acc = voxels[key];
            if (acc.count == 0) {
                acc.point = p;
            }
            acc.sum_x += p.x;
            acc.sum_y += p.y;
            acc.sum_z += p.z;
            ++acc.count;
        }
        output.reserve(voxels.size());
        for (auto& kv : voxels) {
            Accumulator& acc = kv.second;
            acc.point.x = static_cast<float>(acc.sum_x / acc.count);
            acc.point.y = static_cast<float>(acc.sum_y / acc.count);
            acc.point.z = static_cast<float>(acc.sum_z / acc.count);
            output.push_back(acc.point);
        }
    }

private:
    float leaf_size_x_ = 0.1f;
    float leaf_size_y_ = 0.1f;
    float leaf_size_z_ = 0.1f;
    typename PointCloud<PointT>::Ptr input_;
};

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_INTERNAL_VOXEL_GRID_HPP_
