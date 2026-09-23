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
 * @brief Project / Unproject implementation for camera::Pinhole.
 */

#include "autonomy/localization/atlas/sensor/camera/pinhole.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

Vec2 Pinhole::Project(const Vec3& point_camera) const {
    if (point_camera.z() <= 1e-6) {
        return Vec2::Zero();
    }
    const double inv_z = 1.0 / point_camera.z();
    return Vec2(fx_ * point_camera.x() * inv_z + cx_,
                fy_ * point_camera.y() * inv_z + cy_);
}

Vec3 Pinhole::Unproject(const Vec2& pixel, double depth) const {
    return Vec3((pixel.x() - cx_) * depth / fx_,
                (pixel.y() - cy_) * depth / fy_, depth);
}

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
