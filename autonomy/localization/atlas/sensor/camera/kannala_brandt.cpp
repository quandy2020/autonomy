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
 * @brief Project / Unproject implementation for camera::KannalaBrandt.
 */

#include "autonomy/localization/atlas/sensor/camera/kannala_brandt.hpp"

#include <cmath>

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

KannalaBrandt::KannalaBrandt() {
    params_.assign(8, 0.0);
    params_[0] = params_[1] = 1.0;
    id = next_id++;
}

KannalaBrandt::KannalaBrandt(const std::vector<double>& params)
    : params_(params) {
    if (params_.size() != 8) {
        params_.assign(8, 0.0);
        params_[0] = params_[1] = 1.0;
    }
    id = next_id++;
}

KannalaBrandt::KannalaBrandt(double fx, double fy, double cx, double cy,
                             double k1, double k2, double k3, double k4)
    : params_{fx, fy, cx, cy, k1, k2, k3, k4} {
    id = next_id++;
}

Vec2 KannalaBrandt::Project(const Vec3& point_camera) const {
    const double x = point_camera.x();
    const double y = point_camera.y();
    const double z = point_camera.z();
    const double r2 = x * x + y * y;
    const double r = std::sqrt(r2);
    const double theta = std::atan2(r, z);
    const double theta2 = theta * theta;
    const double theta3 = theta2 * theta;
    const double theta5 = theta3 * theta2;
    const double theta7 = theta5 * theta2;
    const double theta9 = theta7 * theta2;
    const double k1 = params_[4];
    const double k2 = params_[5];
    const double k3 = params_[6];
    const double k4 = params_[7];
    const double theta_d =
        theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;
    const double scale = (r > 1e-8) ? (theta_d / r) : 1.0;
    return Vec2(params_[0] * x * scale + params_[2],
                params_[1] * y * scale + params_[3]);
}

Vec3 KannalaBrandt::Unproject(const Vec2& pixel, double depth) const {
    const double mx = (pixel.x() - params_[2]) / params_[0];
    const double my = (pixel.y() - params_[3]) / params_[1];
    const double scale_xy = std::sqrt(mx * mx + my * my);
    double theta = scale_xy;
    const double k1 = params_[4];
    const double k2 = params_[5];
    const double k3 = params_[6];
    const double k4 = params_[7];
    for (int i = 0; i < 10; ++i) {
        const double theta2 = theta * theta;
        const double theta4 = theta2 * theta2;
        const double theta6 = theta4 * theta2;
        const double theta8 = theta4 * theta4;
        const double f = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 +
                                  k4 * theta8) -
                         scale_xy;
        const double df = 1.0 + 3.0 * k1 * theta2 + 5.0 * k2 * theta4 +
                          7.0 * k3 * theta6 + 9.0 * k4 * theta8;
        if (std::abs(df) < 1e-12) {
            break;
        }
        theta -= f / df;
    }
    const double sin_t = std::sin(theta);
    const double cos_t = std::cos(theta);
    const double inv_r = (scale_xy > 1e-8) ? (1.0 / scale_xy) : 1.0;
    const Vec3 dir(sin_t * mx * inv_r, sin_t * my * inv_r, cos_t);
    return dir * depth;
}

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
