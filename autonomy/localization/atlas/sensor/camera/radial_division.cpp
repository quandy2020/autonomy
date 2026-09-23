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
 * @brief Project / Unproject implementation for camera::RadialDivision.
 */

#include "autonomy/localization/atlas/sensor/camera/radial_division.hpp"

#include <cmath>

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

Vec2 RadialDivision::Project(const Vec3& point_camera) const {
    if (point_camera.z() <= 1e-6) {
        return Vec2::Zero();
    }
    const double inv_z = 1.0 / point_camera.z();
    const double x = point_camera.x() * inv_z;
    const double y = point_camera.y() * inv_z;
    const double r2 = x * x + y * y;
    const double denom = 1.0 + k_ * r2;
    if (std::abs(denom) < 1e-12) {
        return Vec2::Zero();
    }
    return Vec2(fx_ * x / denom + cx_, fy_ * y / denom + cy_);
}

Vec3 RadialDivision::Unproject(const Vec2& pixel, double depth) const {
    const double xd = (pixel.x() - cx_) / fx_;
    const double yd = (pixel.y() - cy_) / fy_;
    const double rd2 = xd * xd + yd * yd;
    if (std::abs(k_) > 1e-12 && rd2 > 1e-16) {
        // rd = ru/(1+k ru^2) ⇒ ru = 2 rd / (1 + sqrt(1 - 4 k rd^2))
        const double disc = 1.0 - 4.0 * k_ * rd2;
        if (disc >= 0.0) {
            const double ru =
                (2.0 * std::sqrt(rd2)) / (1.0 + std::sqrt(disc));
            const double scale = ru / std::sqrt(rd2);
            return Vec3(xd * scale * depth, yd * scale * depth, depth);
        }
    }
    return Vec3(xd * depth, yd * depth, depth);
}

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
