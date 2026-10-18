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
 * @file pinhole_camera.hpp
 * @brief Pinhole + optional OpenCV radtan distortion (vikit / FAST-LIVO2).
 */

#ifndef ARGUS_COMMON_CAMERA_PINHOLE_CAMERA_HPP_
#define ARGUS_COMMON_CAMERA_PINHOLE_CAMERA_HPP_

#include <cmath>

#include <opencv2/core.hpp>

#include "Eigen/Core"

#include "argus/common/camera/abstract_camera.hpp"

namespace argus {
namespace camera {

/**
 * @brief Pinhole camera with optional OpenCV radial-tangential distortion.
 *
 * Matches FAST-LIVO2 / rpg_vikit PinholeCamera; see @ref AbstractCamera.
 */
class PinholeCamera : public AbstractCamera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct with absolute-pixel intrinsics and optional radtan
     * distortion.
     * @param width Image width (pixels, before scale).
     * @param height Image height (pixels, before scale).
     * @param scale Resize factor applied to size and intrinsics.
     * @param fx Focal length u (pixels).
     * @param fy Focal length v (pixels).
     * @param cx Principal point u (pixels).
     * @param cy Principal point v (pixels).
     * @param d0 Radial k1 (default 0).
     * @param d1 Radial k2 (default 0).
     * @param d2 Tangential p1 (default 0).
     * @param d3 Tangential p2 (default 0).
     * @param d4 Radial k3 (default 0).
     */
    PinholeCamera(double width, double height, double scale, double fx,
                  double fy, double cx, double cy, double d0 = 0.0,
                  double d1 = 0.0, double d2 = 0.0, double d3 = 0.0,
                  double d4 = 0.0);

    ~PinholeCamera() override = default;

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
    /** @return 3×3 intrinsic matrix K. */
    const Eigen::Matrix3d& K() const {
        return K_;
    }
    /** @return Inverse of K. */
    const Eigen::Matrix3d& K_inv() const {
        return K_inv_;
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
    /** @return Radial distortion k1. */
    double d0() const {
        return d_[0];
    }
    /** @return Radial distortion k2. */
    double d1() const {
        return d_[1];
    }
    /** @return Tangential distortion p1. */
    double d2() const {
        return d_[2];
    }
    /** @return Tangential distortion p2. */
    double d3() const {
        return d_[3];
    }
    /** @return Radial distortion k3. */
    double d4() const {
        return d_[4];
    }

    /**
     * @brief Rectify a distorted image using precomputed OpenCV undistort maps.
     * @param raw Input distorted image.
     * @param rectified Output rectified image (same type as @p raw).
     */
    void undistortImage(const cv::Mat& raw, cv::Mat& rectified) const;

private:
    double fx_ = 1.0;
    double fy_ = 1.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    bool distortion_ = false;
    double d_[5] = {0, 0, 0, 0, 0};
    cv::Mat cvK_;
    cv::Mat cvD_;
    cv::Mat undist_map1_;
    cv::Mat undist_map2_;
    Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d K_inv_ = Eigen::Matrix3d::Identity();
};

}  // namespace camera
}  // namespace argus

#endif  // ARGUS_COMMON_CAMERA_PINHOLE_CAMERA_HPP_
