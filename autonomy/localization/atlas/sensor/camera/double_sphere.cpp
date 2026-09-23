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
 * @brief Project / Unproject implementation for camera::DoubleSphere.
 */

#include "autonomy/localization/atlas/sensor/camera/double_sphere.hpp"

#include <cmath>

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

Vec2 DoubleSphere::Project(const Vec3& point_camera) const {
    const double x = point_camera.x();
    const double y = point_camera.y();
    const double z = point_camera.z();
    const double d1 = point_camera.norm();
    if (d1 < 1e-12) {
        return Vec2::Zero();
    }
    const double z_s = xi_ * d1 + z;
    const double d2 = std::sqrt(x * x + y * y + z_s * z_s);
    const double denom = alpha_ * d2 + (1.0 - alpha_) * z_s;
    if (std::abs(denom) < 1e-12) {
        return Vec2::Zero();
    }
    return Vec2(fx_ * x / denom + cx_, fy_ * y / denom + cy_);
}

Vec3 DoubleSphere::Unproject(const Vec2& pixel, double depth) const {
    const double mx = (pixel.x() - cx_) / fx_;
    const double my = (pixel.y() - cy_) / fy_;
    const double r2 = mx * mx + my * my;
    // Basalt double-sphere unproject.
    double mz = 0.0;
    if (alpha_ > 0.5) {
        const double disc =
            1.0 - (2.0 * alpha_ - 1.0) * r2;
        if (disc < 0.0) {
            return Vec3::Zero();
        }
        mz = (1.0 - alpha_ * alpha_ * r2) /
             (alpha_ * std::sqrt(disc) + 1.0 - alpha_);
    } else {
        mz = (1.0 - alpha_ * alpha_ * r2) /
             (alpha_ * std::sqrt(1.0 - (2.0 * alpha_ - 1.0) * r2) + 1.0 -
              alpha_);
    }
    const double mz2 = mz * mz;
    const double xi2 = xi_ * xi_;
    const double disc2 =
        mz2 + (1.0 - xi2) * r2;
    if (disc2 < 0.0) {
        return Vec3::Zero();
    }
    const double scale =
        (mz * xi_ + std::sqrt(disc2)) / (mz2 + r2);
    const Vec3 dir(scale * mx, scale * my, scale * mz - xi_);
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
