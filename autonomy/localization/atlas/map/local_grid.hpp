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
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "autonomy/localization/atlas/type.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace map {

/** One voxel cell in camera_link / map frame (ROS FLU). */
struct Cell3D {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    /** Packed RGB (optional; 0 = none). */
    uint32_t rgb = 0;
};

/**
 * Local occupancy snapshot for one RGB-D frame (RTAB-Map LocalGrid analogue).
 * Cells are in the sensor/camera_link frame before global assembly.
 */
struct LocalGrid {
    std::vector<Cell3D> ground;
    std::vector<Cell3D> obstacles;
    std::vector<Cell3D> empty;
    float cell_size = 0.05f;
    Eigen::Vector3f view_point = Eigen::Vector3f::Zero();
    /** Camera→world pose used when this grid was created (OpenCV optical). */
    Mat44_t T_wc_opencv = Mat44_t::Identity();
    double timestamp_sec = 0.0;
    uint64_t id = 0;
    /** Atlas keyframe id when keyed to SLAM graph (0 = unlinked). */
    unsigned int keyframe_id = 0;

    bool empty_grid() const {
        return ground.empty() && obstacles.empty() && empty.empty();
    }
};

using LocalGridCache = std::unordered_map<uint64_t, LocalGrid>;

}  // namespace map
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
