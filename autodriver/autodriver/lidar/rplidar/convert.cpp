/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autodriver/lidar/rplidar/convert.hpp"

#include <cmath>
#include <limits>

namespace autodriver {
namespace lidar {
namespace rplidar {

#ifdef AUTODRIVER_HAVE_RPLIDAR

automsgs::msgs::sensor_msgs::LaserScan NodesToLaserScan(
    const sl_lidar_response_measurement_node_hq_t* nodes, std::size_t count,
    float angle_min_rad, float angle_max_rad, const ConvertOptions& opt) {
    automsgs::msgs::sensor_msgs::LaserScan scan;
    scan.mutable_header()->set_frame_id(opt.frame_id);
    if (nodes == nullptr || count < 2) {
        return scan;
    }

    const bool reversed = (angle_max_rad > angle_min_rad);
    if (reversed) {
        scan.set_angle_min(static_cast<float>(M_PI) - angle_max_rad);
        scan.set_angle_max(static_cast<float>(M_PI) - angle_min_rad);
    } else {
        scan.set_angle_min(static_cast<float>(M_PI) - angle_min_rad);
        scan.set_angle_max(static_cast<float>(M_PI) - angle_max_rad);
    }
    scan.set_angle_increment(
        (scan.angle_max() - scan.angle_min()) /
        static_cast<float>(count - 1));
    scan.set_scan_time(static_cast<float>(opt.scan_time_s));
    scan.set_time_increment(
        static_cast<float>(opt.scan_time_s / static_cast<double>(count - 1)));
    scan.set_range_min(opt.range_min);
    scan.set_range_max(opt.range_max);

    const bool reverse_data =
        (!opt.inverted && reversed) || (opt.inverted && !reversed);
    scan.mutable_ranges()->Resize(static_cast<int>(count), 0.f);
    scan.mutable_intensities()->Resize(static_cast<int>(count), 0.f);

    for (std::size_t i = 0; i < count; ++i) {
        const float range =
            static_cast<float>(nodes[i].dist_mm_q2) / 4.0f / 1000.0f;
        const float intensity =
            static_cast<float>(nodes[i].quality >> 2);
        const std::size_t out = reverse_data ? (count - 1 - i) : i;
        if (range == 0.0f) {
            (*scan.mutable_ranges())[static_cast<int>(out)] =
                std::numeric_limits<float>::infinity();
        } else {
            (*scan.mutable_ranges())[static_cast<int>(out)] = range;
        }
        (*scan.mutable_intensities())[static_cast<int>(out)] = intensity;
    }
    return scan;
}

#endif  // AUTODRIVER_HAVE_RPLIDAR

}  // namespace rplidar
}  // namespace lidar
}  // namespace autodriver
