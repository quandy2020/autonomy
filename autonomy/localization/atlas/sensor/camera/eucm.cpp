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
 * @brief Project / Unproject implementation for camera::Eucm.
 */

#include "autonomy/localization/atlas/sensor/camera/eucm.hpp"

#include <cmath>

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

Vec2 Eucm::Project(const Vec3& point_camera) const {
    const double x = point_camera.x();
    const double y = point_camera.y();
    const double z = point_camera.z();
    const double d = std::sqrt(beta_ * (x * x + y * y) + z * z);
    const double denom = alpha_ * d + (1.0 - alpha_) * z;
    if (std::abs(denom) < 1e-12) {
        return Vec2::Zero();
    }
    return Vec2(fx_ * x / denom + cx_, fy_ * y / denom + cy_);
}

Vec3 Eucm::Unproject(const Vec2& pixel, double depth) const {
    const double mx = (pixel.x() - cx_) / fx_;
    const double my = (pixel.y() - cy_) / fy_;
    const double r2 = mx * mx + my * my;
    const double gamma = 1.0 - alpha_;
    // Basalt EUCM unproject.
    const double mz_num = 1.0 - alpha_ * alpha_ * beta_ * r2;
    const double mz_den =
        alpha_ * std::sqrt(std::max(
                     0.0, 1.0 - (2.0 * alpha_ - 1.0) * beta_ * r2)) +
        gamma;
    if (std::abs(mz_den) < 1e-12) {
        return Vec3::Zero();
    }
    const double mz = mz_num / mz_den;
    const Vec3 dir(mx, my, mz);
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
