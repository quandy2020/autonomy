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

#include "argus/common/camera/equidistant_camera.hpp"

#include <cmath>

namespace argus {
namespace camera {

EquidistantCamera::EquidistantCamera(double width, double height, double scale,
                                     double fx, double fy, double cx, double cy,
                                     double k1, double k2, double k3, double k4)
    : AbstractCamera(static_cast<int>(width * scale),
                     static_cast<int>(height * scale), scale),
      fx_(fx * scale),
      fy_(fy * scale),
      cx_(cx * scale),
      cy_(cy * scale),
      distortion_(std::fabs(k1) > 1e-7),
      k1_(k1),
      k2_(k2),
      k3_(k3),
      k4_(k4) {}

Eigen::Vector3d EquidistantCamera::cam2world(const double& u,
                                             const double& v) const {
    Eigen::Vector3d xyz;
    if (!distortion_) {
        xyz[0] = (u - cx_) / fx_;
        xyz[1] = (v - cy_) / fy_;
        xyz[2] = 1.0;
    } else {
        double x = (u - cx_) / fx_;
        double y = (v - cy_) / fy_;
        const double thetad = std::sqrt(x * x + y * y);
        double theta = thetad;
        for (int i = 0; i < 5; ++i) {
            const double theta2 = theta * theta;
            const double theta4 = theta2 * theta2;
            const double theta6 = theta4 * theta2;
            const double theta8 = theta4 * theta4;
            theta = thetad / (1.0 + k1_ * theta2 + k2_ * theta4 + k3_ * theta6 +
                              k4_ * theta8);
        }
        if (thetad > 1e-8) {
            const double scaling = std::tan(theta) / thetad;
            x *= scaling;
            y *= scaling;
        }
        xyz[0] = x;
        xyz[1] = y;
        xyz[2] = 1.0;
    }
    return xyz.normalized();
}

Eigen::Vector3d EquidistantCamera::cam2world(const Eigen::Vector2d& uv) const {
    return cam2world(uv[0], uv[1]);
}

Eigen::Vector2d EquidistantCamera::world2cam(const Eigen::Vector3d& xyz) const {
    return world2cam(Project2d(xyz));
}

Eigen::Vector2d EquidistantCamera::world2cam(const Eigen::Vector2d& uv) const {
    Eigen::Vector2d px;
    if (!distortion_) {
        px[0] = fx_ * uv[0] + cx_;
        px[1] = fy_ * uv[1] + cy_;
    } else {
        const double r = uv.norm();
        if (r < 1e-8) {
            return Eigen::Vector2d(fx_ * uv[0] + cx_, fy_ * uv[1] + cy_);
        }
        const double theta = std::atan(r);
        const double thetad = thetad_from_theta(theta);
        const double scaling = thetad / r;
        px[0] = uv[0] * scaling * fx_ + cx_;
        px[1] = uv[1] * scaling * fy_ + cy_;
    }
    return px;
}

}  // namespace camera
}  // namespace argus
