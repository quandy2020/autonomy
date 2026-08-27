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
#include <vector>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autonomy/localization/atlas/map/local_grid.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace map {

/**
 * Assemble LocalGrids into layered global clouds (RTAB-Map CloudMap):
 * ground / obstacles / empty, with persistent voxel hash (no full rebuild).
 */
class CloudMap {
public:
    enum class Layer { kMerged = 0, kGround = 1, kObstacles = 2, kEmpty = 3 };

    struct Options {
        float voxel_size = 0.05f;
        bool include_ground = true;
        bool include_obstacles = true;
        bool include_empty = false;
        /** Paint default colors when RGB missing (RTAB-Map: green/red). */
        bool colorize_layers = true;
        uint32_t ground_rgb = 0x00c800;      // green
        uint32_t obstacle_rgb = 0xc80000;    // red
        uint32_t empty_rgb = 0x808080;       // gray
        int max_points = 500000;
    };

    explicit CloudMap(Options options);

    void Clear();
    void Integrate(const LocalGrid& local);

    bool ToMessage(const std::string& frame_id, double timestamp_sec,
                   Layer layer,
                   automsgs::msgs::sensor_msgs::PointCloud2* msg) const;

    std::size_t size() const;
    std::size_t ground_size() const;
    std::size_t obstacle_size() const;

private:
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
    using VoxelHash = std::unordered_map<VoxelKey, Cell3D, VoxelKeyHash>;

    void InsertLayer(VoxelHash* dst, const std::vector<Cell3D>& layer,
                     const Mat44_t& T_map_link, uint32_t default_rgb);
    static bool FillMessage(const VoxelHash& voxels, const std::string& frame_id,
                            double timestamp_sec, int max_points,
                            automsgs::msgs::sensor_msgs::PointCloud2* msg);

    Options options_;
    mutable std::mutex mutex_;
    VoxelHash ground_;
    VoxelHash obstacles_;
    VoxelHash empty_;
};

}  // namespace map
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
