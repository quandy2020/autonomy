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
 * @file
 * @brief Project / Unproject implementation for camera::Equirectangular.
 */

#include "autonomy/localization/atlas/sensor/camera/equirectangular.hpp"

#include <cmath>

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

Vec2 Equirectangular::Project(const Vec3& point_camera) const {
    const double d = point_camera.norm();
    if (d < 1e-12) {
        return Vec2::Zero();
    }
    const double lon = std::atan2(point_camera.x(), point_camera.z());
    const double lat = std::asin(std::max(-1.0, std::min(1.0, point_camera.y() / d)));
    return Vec2(fx_ * lon + cx_, fy_ * lat + cy_);
}

Vec3 Equirectangular::Unproject(const Vec2& pixel, double depth) const {
    const double lon = (pixel.x() - cx_) / fx_;
    const double lat = (pixel.y() - cy_) / fy_;
    const double cos_lat = std::cos(lat);
    const Vec3 dir(cos_lat * std::sin(lon), std::sin(lat),
                   cos_lat * std::cos(lon));
    return dir * depth;
}

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
