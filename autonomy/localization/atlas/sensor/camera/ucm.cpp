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
 * @brief Project / Unproject implementation for camera::Ucm.
 */

#include "autonomy/localization/atlas/sensor/camera/ucm.hpp"

#include <cmath>

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

Vec2 Ucm::Project(const Vec3& point_camera) const {
    const double d = point_camera.norm();
    if (d < 1e-12) {
        return Vec2::Zero();
    }
    const double denom = xi_ * d + point_camera.z();
    if (std::abs(denom) < 1e-12) {
        return Vec2::Zero();
    }
    return Vec2(fx_ * point_camera.x() / denom + cx_,
                fy_ * point_camera.y() / denom + cy_);
}

Vec3 Ucm::Unproject(const Vec2& pixel, double depth) const {
    // Basalt/Kalibr Mei UCM unproject → unit bearing, then × depth.
    const double mx = (pixel.x() - cx_) / fx_;
    const double my = (pixel.y() - cy_) / fy_;
    const double r2 = mx * mx + my * my;
    const double xi2 = xi_ * xi_;
    const double disc = 1.0 + (1.0 - xi2) * r2;
    if (disc < 0.0) {
        return Vec3::Zero();
    }
    const double factor = (xi_ + std::sqrt(disc)) / (r2 + 1.0);
    const Vec3 dir(factor * mx, factor * my, factor - xi_);
    const double n = dir.norm();
    if (n < 1e-12) {
        return Vec3::Zero();
    }
    return dir * (depth / n);
}

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
