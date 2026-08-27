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

#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autonomy/localization/atlas/map/local_grid.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace map {

/**
 * Elevation / height map (RTAB-Map GridMap style): per XY cell keep max Z.
 * Publishes PointCloud2 surface + OccupancyGrid with height encoded 0–100.
 */
class ElevationMap {
public:
    struct Options {
        float cell_size = 0.05f;
        bool include_ground = true;
        bool include_obstacles = true;
        uint32_t default_rgb = 0x4682b4;  // steel blue
        int max_cells = 500000;
        int border_cells = 2;
        int max_width = 2000;
        int max_height = 2000;
    };

    explicit ElevationMap(Options options);

    void Clear();
    void Integrate(const LocalGrid& local);

    bool ToCloudMessage(const std::string& frame_id, double timestamp_sec,
                        automsgs::msgs::sensor_msgs::PointCloud2* msg) const;

    /** Height encoded as 0–100 over [min_z, max_z]; -1 unknown. */
    bool ToGridMessage(const std::string& frame_id, double timestamp_sec,
                       automsgs::msgs::map_msgs::OccupancyGrid* msg) const;

    std::size_t size() const;

private:
    struct CellKey {
        int ix = 0;
        int iy = 0;
        bool operator==(const CellKey& o) const {
            return ix == o.ix && iy == o.iy;
        }
    };
    struct CellKeyHash {
        std::size_t operator()(const CellKey& k) const {
            return (static_cast<std::size_t>(k.ix) * 73856093u) ^
                   (static_cast<std::size_t>(k.iy) * 19349663u);
        }
    };
    struct Cell {
        float z = -std::numeric_limits<float>::infinity();
        uint32_t rgb = 0;
        uint64_t node_id = 0;
    };

    void Insert(float x, float y, float z, uint32_t rgb, uint64_t node_id);

    Options options_;
    mutable std::mutex mutex_;
    std::unordered_map<CellKey, Cell, CellKeyHash> cells_;
    float min_z_ = std::numeric_limits<float>::infinity();
    float max_z_ = -std::numeric_limits<float>::infinity();
};

}  // namespace map
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
