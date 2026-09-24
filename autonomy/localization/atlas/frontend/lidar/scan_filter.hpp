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
 * @file scan_filter.hpp
 * @brief Body-frame lidar filters used before Ceres scan matching.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_LIDAR_SCAN_FILTER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_LIDAR_SCAN_FILTER_HPP_

#include <cmath>

#include "autonomy/localization/atlas/common/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @brief Drop blind-range points, optionally stride and clip height.
 * @param[in,out] scan Body-frame cloud. Empty input is left empty.
 * @param blind Minimum range in meters. Non-positive disables the test.
 * @param stride Keep one point out of this many. Values below 2 keep all.
 * @param z_min / z_max Inclusive body-frame height band. Ignored when
 *        `z_max <= z_min`.
 */
inline void FilterScan(PointCloud* scan, double blind, int stride, double z_min,
                       double z_max) {
    if (scan == nullptr || scan->empty()) {
        return;
    }
    const double blind2 = blind > 0.0 ? blind * blind : 0.0;
    const bool clip_height = z_max > z_min;
    const int step = stride > 1 ? stride : 1;
    PointCloud kept;
    kept.reserve(scan->size() / static_cast<std::size_t>(step) + 1);
    for (std::size_t i = 0; i < scan->size(); i += static_cast<std::size_t>(step)) {
        const auto& point = (*scan)[i];
        if (blind2 > 0.0) {
            const double range2 = static_cast<double>(point.x) * point.x +
                                  static_cast<double>(point.y) * point.y +
                                  static_cast<double>(point.z) * point.z;
            if (range2 < blind2) {
                continue;
            }
        }
        if (clip_height && (point.z < z_min || point.z > z_max)) {
            continue;
        }
        kept.push_back(point);
    }
    *scan = std::move(kept);
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_LIDAR_SCAN_FILTER_HPP_
