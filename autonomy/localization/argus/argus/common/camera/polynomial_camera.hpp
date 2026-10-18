/*
 * Copyright 2026 The Openbot Authors
 * Port of rpg_vikit PolynomialCamera (Chunran Zheng / FAST-LIVO2).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

/**
 * @file polynomial_camera.hpp
 * @brief Pinhole + polynomial angular distortion (vikit PolynomialCamera).
 */

#ifndef ARGUS_COMMON_CAMERA_POLYNOMIAL_CAMERA_HPP_
#define ARGUS_COMMON_CAMERA_POLYNOMIAL_CAMERA_HPP_

#include <cmath>

#include "Eigen/Core"

#include "argus/common/camera/abstract_camera.hpp"

namespace argus {
namespace camera {

/**
 * @brief Pinhole intrinsics with polynomial angular distortion (rpg_vikit).
 *
 * Distortion maps incidence angle θ to θ_d via a polynomial in θ; see
 * @ref AbstractCamera for projection semantics.
 */
class PolynomialCamera : public AbstractCamera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct with absolute-pixel intrinsics and optional image scale.
     * @param width Image width (pixels).
     * @param height Image height (pixels).
     * @param scale Pyramid scale factor.
     * @param fx Focal length u (pixels).
     * @param fy Focal length v (pixels).
     * @param cx Principal point u (pixels).
     * @param cy Principal point v (pixels).
     * @param skew Skew coefficient (default 0).
     * @param k2..k7 Polynomial distortion coefficients (default 0).
     */
    PolynomialCamera(double width, double height, double scale, double fx,
                     double fy, double cx, double cy, double skew = 0.0,
                     double k2 = 0.0, double k3 = 0.0, double k4 = 0.0,
                     double k5 = 0.0, double k6 = 0.0, double k7 = 0.0);

    ~PolynomialCamera() override = default;

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
    /** @return Image skew coefficient. */
    double skew() const {
        return skew_;
    }

    /**
     * @brief Distorted angle θ_d from true incidence angle θ.
     * @param theta Incidence angle (radians).
     * @return θ_d after polynomial distortion.
     */
    double thetad_from_theta(double theta) const {
        const double theta2 = theta * theta;
        const double theta3 = theta2 * theta;
        const double theta4 = theta3 * theta;
        const double theta5 = theta4 * theta;
        const double theta6 = theta5 * theta;
        const double theta7 = theta6 * theta;
        return theta + k2_ * theta2 + k3_ * theta3 + k4_ * theta4 +
               k5_ * theta5 + k6_ * theta6 + k7_ * theta7;
    }

    /**
     * @brief Derivative d(θ_d)/dθ at θ.
     * @param theta Incidence angle (radians).
     * @return Derivative of @ref thetad_from_theta.
     */
    double deriv_thetad_from_theta(double theta) const {
        const double theta2 = theta * theta;
        const double theta3 = theta2 * theta;
        const double theta4 = theta3 * theta;
        const double theta5 = theta4 * theta;
        const double theta6 = theta5 * theta;
        return 1.0 + 2.0 * k2_ * theta + 3.0 * k3_ * theta2 +
               4.0 * k4_ * theta3 + 5.0 * k5_ * theta4 + 6.0 * k6_ * theta5 +
               7.0 * k7_ * theta6;
    }

    /**
     * @brief Jacobian of world2cam(uv) w.r.t. uv on the unit plane.
     * @param uv Point on unit plane (z = 1).
     * @return 2×2 image Jacobian.
     */
    Eigen::Matrix2d jacobian_2x2(const Eigen::Vector2d& uv) const;
    /**
     * @brief Jacobian of world2cam(xyz) w.r.t. 3D camera point.
     * @param p 3D point in camera frame.
     * @return 2×3 projection Jacobian.
     */
    Eigen::Matrix<double, 2, 3> jacobian_2x3(const Eigen::Vector3d& p) const;

private:
    double fx_ = 1.0;
    double fy_ = 1.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    double skew_ = 0.0;
    bool distortion_ = false;
    double k2_ = 0.0;
    double k3_ = 0.0;
    double k4_ = 0.0;
    double k5_ = 0.0;
    double k6_ = 0.0;
    double k7_ = 0.0;
};

}  // namespace camera
}  // namespace argus

#endif  // ARGUS_COMMON_CAMERA_POLYNOMIAL_CAMERA_HPP_
