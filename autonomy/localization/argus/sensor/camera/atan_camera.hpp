/*
 * Copyright 2026 The Openbot Authors
 * Port of rpg_vikit ATANCamera (FOV / Devernay–Faugeras), ROS-free.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

/**
 * @file atan_camera.hpp
 * @brief FOV distortion model (vikit ATANCamera / PTAM).
 *
 * Absolute-pixel constructor (Argus style, with optional `scale`).
 * `FromNormalized` matches rpg_vikit: fx,fy,cx,cy as fractions of W/H.
 */

#ifndef ARGUS_COMMON_CAMERA_ATAN_CAMERA_HPP_
#define ARGUS_COMMON_CAMERA_ATAN_CAMERA_HPP_

#include <cmath>

#include "Eigen/Core"

#include "argus/common/camera/abstract_camera.hpp"

namespace argus {
namespace camera {

/**
 * @brief FOV (ATAN) distortion model from PTAM / rpg_vikit ATANCamera.
 *
 * Uses Devernay–Faugeras wide-angle mapping with parameter @c s; see
 * @ref AbstractCamera for projection semantics.
 */
class ATANCamera : public AbstractCamera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct with absolute-pixel intrinsics and optional image scale.
     * @param width Image width (pixels).
     * @param height Image height (pixels).
     * @param scale Pyramid scale factor.
     * @param fx Focal length u (pixels, pre-scale).
     * @param fy Focal length v (pixels, pre-scale).
     * @param cx Principal point u (pixels).
     * @param cy Principal point v (pixels).
     * @param s FOV distortion parameter (0 = pinhole on unit plane).
     */
    ATANCamera(double width, double height, double scale, double fx, double fy,
               double cx, double cy, double s);

    /**
     * @brief rpg_vikit factory: fx, fy, cx, cy normalized by width/height.
     * @param width Image width (pixels).
     * @param height Image height (pixels).
     * @param fx_n Normalized focal u (fx / width).
     * @param fy_n Normalized focal v (fy / height).
     * @param cx_n Normalized principal u; internally cx = cx_n * width - 0.5.
     * @param cy_n Normalized principal v.
     * @param s FOV distortion parameter.
     * @return Camera with absolute-pixel intrinsics.
     */
    static ATANCamera FromNormalized(double width, double height, double fx_n,
                                     double fy_n, double cx_n, double cy_n,
                                     double s);

    ~ATANCamera() override = default;

    /** @param x Pixel u coordinate. @param y Pixel v coordinate. @return Unit
     * bearing in camera frame. */
    Eigen::Vector3d cam2world(const double& x, const double& y) const override;
    /** @param px Pixel coordinates. @return Unit bearing in camera frame. */
    Eigen::Vector3d cam2world(const Eigen::Vector2d& px) const override;
    /** @param xyz_c 3D point in camera frame. @return Pixel coordinates. */
    Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz_c) const override;
    /** @param uv Point on unit plane (z = 1). @return Pixel coordinates. */
    Eigen::Vector2d world2cam(const Eigen::Vector2d& uv) const override;

    /** @return Visual reprojection error scale (fx). */
    double errorMultiplier2() const override {
        return fx_;
    }
    /** @return Visual reprojection error scale (4 fx fy). */
    double errorMultiplier() const override {
        return 4.0 * fx_ * fy_;
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
    /** @return FOV distortion parameter s. */
    double s() const {
        return s_;
    }

private:
    ATANCamera(int width, int height, double scale, double fx, double fy,
               double cx, double cy, double s, bool /*tag*/);

    double rtrans_factor(double r) const {
        if (r < 0.001 || s_ == 0.0) {
            return 1.0;
        }
        return (s_inv_ * std::atan(r * tans_) / r);
    }

    double invrtrans(double r) const {
        if (s_ == 0.0) {
            return r;
        }
        return (std::tan(r * s_) * tans_inv_);
    }

    double fx_ = 1.0;
    double fy_ = 1.0;
    double fx_inv_ = 1.0;
    double fy_inv_ = 1.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    double s_ = 0.0;
    double s_inv_ = 0.0;
    double tans_ = 0.0;
    double tans_inv_ = 0.0;
    bool distortion_ = false;
};

}  // namespace camera
}  // namespace argus

#endif  // ARGUS_COMMON_CAMERA_ATAN_CAMERA_HPP_
