/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#include "autonomy/map/costmap_3d/map_io.hpp"

namespace autonomy {
namespace map {
namespace costmap_3d {


bool LoadPlyFile(const std::string& ply_filename, commsgs::sensor_msgs::PointCloud& cloud)
{
    if (ply_filename.empty()) {
        return false;
    }

    auto points = common::ReadPly(ply_filename);
    cloud.points.resize(points.size());
    for (auto const& point : points) {
        cloud.points.push_back({
            point.x,
            point.y,
            point.z
        });
    }
    return true;
}

}  // namespace costmap_3d
}  // namespace map
}  // namespace autonomy