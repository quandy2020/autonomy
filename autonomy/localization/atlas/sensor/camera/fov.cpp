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
 * @brief Project / Unproject implementation for camera::Fov.
 */

#include "autonomy/localization/atlas/sensor/camera/fov.hpp"

#include <cmath>

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {
namespace {

constexpr double kEpsW = 1e-6;

}  // namespace

Vec2 Fov::Project(const Vec3& point_camera) const {
    if (point_camera.z() <= 1e-6) {
        return Vec2::Zero();
    }
    const double inv_z = 1.0 / point_camera.z();
    const double x = point_camera.x() * inv_z;
    const double y = point_camera.y() * inv_z;
    const double ru = std::sqrt(x * x + y * y);
    double rd = ru;
    if (std::abs(w_) > kEpsW && ru > 1e-8) {
        rd = std::atan(2.0 * ru * std::tan(w_ * 0.5)) / w_;
    }
    const double scale = (ru > 1e-8) ? (rd / ru) : 1.0;
    return Vec2(fx_ * x * scale + cx_, fy_ * y * scale + cy_);
}

Vec3 Fov::Unproject(const Vec2& pixel, double depth) const {
    const double xd = (pixel.x() - cx_) / fx_;
    const double yd = (pixel.y() - cy_) / fy_;
    const double rd = std::sqrt(xd * xd + yd * yd);
    double ru = rd;
    if (std::abs(w_) > kEpsW && rd > 1e-8) {
        // rd = atan(2 ru tan(w/2)) / w  ⇒  ru = tan(rd w) / (2 tan(w/2))
        ru = std::tan(rd * w_) / (2.0 * std::tan(w_ * 0.5));
    }
    const double scale = (rd > 1e-8) ? (ru / rd) : 1.0;
    return Vec3(xd * scale * depth, yd * scale * depth, depth);
}

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
