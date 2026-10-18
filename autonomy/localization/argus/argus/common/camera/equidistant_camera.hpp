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
 * @file equidistant_camera.hpp
 * @brief Kannala–Brandt equidistant fisheye (FAST-LIVO2 EquidistantCamera).
 */

#ifndef ARGUS_COMMON_CAMERA_EQUIDISTANT_CAMERA_HPP_
#define ARGUS_COMMON_CAMERA_EQUIDISTANT_CAMERA_HPP_

#include <cmath>

#include "Eigen/Core"

#include "argus/common/camera/abstract_camera.hpp"

namespace argus {
namespace camera {

/**
 * @brief Kannala–Brandt equidistant fisheye with optional k1–k4 angle
 * distortion.
 *
 * Bearing rays use the equidistant relation between incidence angle θ and
 * distorted radius θ_d on the image plane; see @ref AbstractCamera for
 * projection semantics.
 */
class EquidistantCamera : public AbstractCamera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @param width Image width (pixels).
     * @param height Image height (pixels).
     * @param scale Pyramid scale factor.
     * @param fx Focal length u (pixels).
     * @param fy Focal length v (pixels).
     * @param cx Principal point u (pixels).
     * @param cy Principal point v (pixels).
     * @param k1 First equidistant distortion coefficient (default 0).
     * @param k2 Second coefficient (default 0).
     * @param k3 Third coefficient (default 0).
     * @param k4 Fourth coefficient (default 0).
     */
    EquidistantCamera(double width, double height, double scale, double fx,
                      double fy, double cx, double cy, double k1 = 0.0,
                      double k2 = 0.0, double k3 = 0.0, double k4 = 0.0);

    ~EquidistantCamera() override = default;

    /** @param x Pixel u coordinate. @param y Pixel v coordinate. @return Unit
     * bearing in camera frame. */
    Eigen::Vector3d cam2world(const double& x, const double& y) const override;
    /** @param px Pixel coordinates. @return Unit bearing in camera frame. */
    Eigen::Vector3d cam2world(const Eigen::Vector2d& px) const override;
    /** @param xyz_c 3D point in camera frame. @return Pixel coordinates. */
    Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz_c) const override;
    /** @param uv Point on unit plane (z = 1). @return Pixel coordinates. */
    Eigen::Vector2d world2cam(const Eigen::Vector2d& uv) const override;

    /** @return Visual reprojection error scale (|fx|). */
    double errorMultiplier2() const override {
        return std::fabs(fx_);
    }
    /** @return Visual reprojection error scale (4|fx fy|). */
    double errorMultiplier() const override {
        return std::fabs(4.0 * fx_ * fy_);
    }

    /** @return Focal lengths (fx, fy). */
    Eigen::Vector2d focal_length() const {
        return Eigen::Vector2d(fx_, fy_);
    }

    double fx() const override {
        return fx_;
    }
    double fy() const override {
        return fy_;
    }
    double cx() const override {
        return cx_;
    }
    double cy() const override {
        return cy_;
    }

    /**
     * @brief Distorted equidistant angle θ_d from true incidence angle θ.
     * @param theta Incidence angle (radians).
     * @return θ_d after polynomial in θ² (identity when all k_i are zero).
     */
    double thetad_from_theta(double theta) const {
        const double theta2 = theta * theta;
        const double theta4 = theta2 * theta2;
        const double theta6 = theta4 * theta2;
        const double theta8 = theta4 * theta4;
        return theta * (1.0 + k1_ * theta2 + k2_ * theta4 + k3_ * theta6 +
                        k4_ * theta8);
    }

private:
    double fx_ = 1.0;
    double fy_ = 1.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    bool distortion_ = false;
    double k1_ = 0.0;
    double k2_ = 0.0;
    double k3_ = 0.0;
    double k4_ = 0.0;
};

}  // namespace camera
}  // namespace argus

#endif  // ARGUS_COMMON_CAMERA_EQUIDISTANT_CAMERA_HPP_
