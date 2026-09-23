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
 * @brief Project / Unproject implementation for camera::RadTan.
 */

#include "autonomy/localization/atlas/sensor/camera/radtan.hpp"

#include <cmath>

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

Vec2 RadTan::Distort(double x, double y) const {
    const double x2 = x * x;
    const double y2 = y * y;
    const double r2 = x2 + y2;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double radial = 1.0 + k1_ * r2 + k2_ * r4 + k3_ * r6;
    const double xy2 = 2.0 * x * y;
    return Vec2(x * radial + p1_ * xy2 + p2_ * (r2 + 2.0 * x2),
                y * radial + p2_ * xy2 + p1_ * (r2 + 2.0 * y2));
}

Vec2 RadTan::Project(const Vec3& point_camera) const {
    if (point_camera.z() <= 1e-6) {
        return Vec2::Zero();
    }
    const double inv_z = 1.0 / point_camera.z();
    const Vec2 d = Distort(point_camera.x() * inv_z, point_camera.y() * inv_z);
    return Vec2(fx_ * d.x() + cx_, fy_ * d.y() + cy_);
}

Vec3 RadTan::Unproject(const Vec2& pixel, double depth) const {
    // Iterative undistortion (OpenCV calibrateCamera convention).
    double x = (pixel.x() - cx_) / fx_;
    double y = (pixel.y() - cy_) / fy_;
    const double xd = x;
    const double yd = y;
    for (int i = 0; i < 10; ++i) {
        const double x2 = x * x;
        const double y2 = y * y;
        const double r2 = x2 + y2;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double radial = 1.0 + k1_ * r2 + k2_ * r4 + k3_ * r6;
        const double xy2 = 2.0 * x * y;
        const double dx = p1_ * xy2 + p2_ * (r2 + 2.0 * x2);
        const double dy = p2_ * xy2 + p1_ * (r2 + 2.0 * y2);
        if (std::abs(radial) < 1e-12) {
            break;
        }
        x = (xd - dx) / radial;
        y = (yd - dy) / radial;
    }
    return Vec3(x * depth, y * depth, depth);
}

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
