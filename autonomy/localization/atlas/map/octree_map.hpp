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

#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autonomy/localization/atlas/map/local_grid.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace map {

/**
 * Lightweight OctoMap-style 3D occupancy (voxel hash + log-odds).
 * No liboctomap dependency; publishes PointCloud2 layers like rtabmap_ros:
 *   occupied / ground / empty, plus optional 2D projection OccupancyGrid.
 */
class OctreeMap {
public:
    enum class Layer { kOccupied = 0, kGround = 1, kEmpty = 2 };

    struct Options {
        float voxel_size = 0.05f;
        float prob_hit = 0.7f;
        float prob_miss = 0.4f;
        float occupancy_thr = 0.5f;
        float clamping_min = 0.12f;
        float clamping_max = 0.97f;
        bool colorize = true;
        uint32_t ground_rgb = 0x00c800;
        uint32_t obstacle_rgb = 0xc80000;
        uint32_t empty_rgb = 0x808080;
        int max_voxels = 500000;
    };

    explicit OctreeMap(Options options);

    void Clear();
    void Integrate(const LocalGrid& local);

    bool ToCloudMessage(const std::string& frame_id, double timestamp_sec,
                        Layer layer,
                        automsgs::msgs::sensor_msgs::PointCloud2* msg) const;

    /** Project occupied voxels to XY OccupancyGrid (rtabmap octomap_grid). */
    bool ToProjectionMessage(
        const std::string& frame_id, double timestamp_sec,
        automsgs::msgs::map_msgs::OccupancyGrid* msg) const;

    std::size_t occupied_size() const;
    std::size_t ground_size() const;
    std::size_t empty_size() const;

private:
    enum class CellType : uint8_t { kUnknown = 0, kEmpty, kGround, kObstacle };

    struct VoxelKey {
        int ix = 0;
        int iy = 0;
        int iz = 0;
        bool operator==(const VoxelKey& o) const {
            return ix == o.ix && iy == o.iy && iz == o.iz;
        }
    };
    struct VoxelKeyHash {
        std::size_t operator()(const VoxelKey& k) const {
            return (static_cast<std::size_t>(k.ix) * 73856093u) ^
                   (static_cast<std::size_t>(k.iy) * 19349663u) ^
                   (static_cast<std::size_t>(k.iz) * 83492791u);
        }
    };
    struct Voxel {
        float log_odds = 0.f;
        CellType type = CellType::kUnknown;
        uint32_t rgb = 0;
    };

    float LogOdds(float p) const;
    float ClampLogOdds(float l) const;
    float Prob(float log_odds) const;
    void UpdateVoxel(const Eigen::Vector3f& p, float delta, CellType type,
                     uint32_t rgb);

    Options options_;
    mutable std::mutex mutex_;
    std::unordered_map<VoxelKey, Voxel, VoxelKeyHash> voxels_;
};

}  // namespace map
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
