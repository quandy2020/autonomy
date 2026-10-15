/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#include "autonomy/control/controller/neupan_controller/obstacle_extractor.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

int ExtractObstaclePointsFromLaserScan(
    const automsgs::msgs::sensor_msgs::LaserScan& laser_scan,
    const LaserExtrinsic& extrinsic, int max_point_count,
    Eigen::Matrix2Xd* points_out) {
    if (points_out == nullptr) {
        return 0;
    }

    const double range_min =
        extrinsic.range_min > 0.0
            ? extrinsic.range_min
            : (laser_scan.range_min() > 0.0 ? laser_scan.range_min() : 0.05);
    const double range_max =
        extrinsic.range_max > 0.0
            ? extrinsic.range_max
            : (laser_scan.range_max() > 0.0 ? laser_scan.range_max() : 30.0);

    const int range_count = laser_scan.ranges_size();
    if (range_count <= 0) {
        points_out->resize(2, 0);
        return 0;
    }

    const double angle_min = laser_scan.angle_min();
    const double angle_increment =
        laser_scan.angle_increment() != 0.0
            ? laser_scan.angle_increment()
            : (range_count > 1 ? (laser_scan.angle_max() - angle_min) /
                                     static_cast<double>(range_count - 1)
                               : 0.0);

    const int stride =
        (max_point_count > 0 && range_count > max_point_count)
            ? std::max(1, (range_count + max_point_count - 1) / max_point_count)
            : 1;
    const int capacity =
        max_point_count > 0
            ? std::min(max_point_count, (range_count + stride - 1) / stride)
            : range_count;

    // Over-allocate then conservativeResize to final N (avoids Point
    // protobufs).
    points_out->resize(2, capacity);
    int written = 0;
    double angle = angle_min;
    const double angle_step = angle_increment * static_cast<double>(stride);
    for (int i = 0; i < range_count && written < capacity;
         i += stride, angle += angle_step) {
        const double range = laser_scan.ranges(i);
        if (!std::isfinite(range) || range < range_min || range > range_max) {
            continue;
        }
        const double px = range * std::cos(angle);
        const double py = range * std::sin(angle);
        (*points_out)(0, written) =
            extrinsic.x + extrinsic.cos_yaw * px - extrinsic.sin_yaw * py;
        (*points_out)(1, written) =
            extrinsic.y + extrinsic.sin_yaw * px + extrinsic.cos_yaw * py;
        ++written;
    }
    points_out->conservativeResize(2, written);
    return written;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
