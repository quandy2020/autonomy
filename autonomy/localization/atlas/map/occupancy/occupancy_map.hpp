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
 * @file occupancy_map.hpp
 * @brief 2D occupancy from lidar rays. Output is automsgs OccupancyGrid.
 *
 * Hits stay occupied. Unknown cells are -1, free cells are 0, occupied is 100.
 * The grid is for navigation, not for pose estimation.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MAP_OCCUPANCY_OCCUPANCY_MAP_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MAP_OCCUPANCY_OCCUPANCY_MAP_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>

#include "autonomy/localization/atlas/common/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

class OccupancyMap {
public:
    struct Options {
        double resolution = 0.1;    ///< Cell size (m).
        double min_z = 0.2;         ///< Body-frame height lower bound (m).
        double max_z = 1.5;         ///< Body-frame height upper bound (m).
        double max_range = 40.0;    ///< Ignore hits farther than this (m).
    };

    OccupancyMap();
    explicit OccupancyMap(Options options);

    void Clear();

    /**
     * @brief Raycast one body-frame scan into the grid.
     * @param T_wb Pose of the sensor body in the map frame.
     * @param body Scan points in the body frame.
     */
    void Integrate(const SE3& T_wb, const PointCloud& body);

    /**
     * @brief Pack the current grid.
     * @param[out] grid OccupancyGrid. Unknown cells are -1.
     * @param frame Header frame, usually `map`.
     * @return false when no cell has been written.
     */
    bool Fill(automsgs::msgs::map_msgs::OccupancyGrid* grid,
              const std::string& frame) const;

    /**
     * @brief Replace cells from a previously filled grid.
     * @param grid OccupancyGrid written by Fill.
     */
    void Load(const automsgs::msgs::map_msgs::OccupancyGrid& grid);

private:
    struct Key {
        int x = 0;
        int y = 0;
        bool operator==(const Key& other) const {
            return x == other.x && y == other.y;
        }
    };
    struct Hash {
        std::size_t operator()(const Key& key) const {
            return (static_cast<std::size_t>(key.x) * 73856093u) ^
                   (static_cast<std::size_t>(key.y) * 19349663u);
        }
    };

    void MarkRay(int x0, int y0, int x1, int y1);

    Options options_;
    std::unordered_map<Key, int8_t, Hash> cells_;
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MAP_OCCUPANCY_OCCUPANCY_MAP_HPP_
