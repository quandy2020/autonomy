/*
 * Copyright 2026 The Openbot Authors
 * Port of rpg_vikit OmniCamera (Scaramuzza OCamCalib), ROS-free.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 */

/**
 * @file omni_camera.hpp
 * @brief Omnidirectional camera (vikit OmniCamera / OCamCalib text file).
 */

#ifndef ARGUS_COMMON_CAMERA_OMNI_CAMERA_HPP_
#define ARGUS_COMMON_CAMERA_OMNI_CAMERA_HPP_

#include <cmath>
#include <string>

#include "Eigen/Core"

#include "argus/common/camera/abstract_camera.hpp"

namespace argus {
namespace camera {

constexpr int kOcamMaxPolLength = 64;

/** @brief Scaramuzza ocam_model (same layout as rpg_vikit). */
struct OcamModel {
    double pol[kOcamMaxPolLength] = {
        0};             /**< @brief Forward polynomial coefficients. */
    int length_pol = 0; /**< @brief Length of @c pol. */
    double invpol[kOcamMaxPolLength] = {
        0};                /**< @brief Inverse polynomial coefficients. */
    int length_invpol = 0; /**< @brief Length of @c invpol. */
    double xc = 0.0;       /**< @brief Optical center row. */
    double yc = 0.0;       /**< @brief Optical center column. */
    double c = 1.0;        /**< @brief Affine stretch. */
    double d = 0.0;        /**< @brief Affine shear term. */
    double e = 0.0;        /**< @brief Affine shear term. */
    int width = 0;         /**< @brief Image width (pixels). */
    int height = 0;        /**< @brief Image height (pixels). */
};

/**
 * @brief Omnidirectional camera from Scaramuzza OCamCalib polynomials.
 *
 * Loads vikit-format calibration text or accepts a parsed @ref OcamModel.
 */
class OmniCamera : public AbstractCamera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Load Scaramuzza calibration text file (vikit format).
     * @param calib_file Path to OCamCalib-style text file.
     */
    explicit OmniCamera(const std::string& calib_file);

    /**
     * @brief Construct from an already-parsed model (tests / YAML).
     * @param model Parsed Scaramuzza parameters.
     */
    explicit OmniCamera(const OcamModel& model);

    ~OmniCamera() override = default;

    /** @return True if calibration loaded and projection is usable. */
    bool valid() const {
        return valid_;
    }

    /** @param x Pixel u coordinate. @param y Pixel v coordinate. @return Unit
     * bearing in camera frame. */
    Eigen::Vector3d cam2world(const double& x, const double& y) const override;
    /** @param px Pixel coordinates. @return Unit bearing in camera frame. */
    Eigen::Vector3d cam2world(const Eigen::Vector2d& px) const override;
    /** @param xyz_c 3D point in camera frame. @return Pixel coordinates. */
    Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz_c) const override;
    /** @param uv Point on unit plane (z = 1). @return Pixel coordinates. */
    Eigen::Vector2d world2cam(const Eigen::Vector2d& uv) const override;

    /** @return Visual reprojection error scale derived from calibration. */
    double errorMultiplier2() const override {
        return std::sqrt(error_multiplier_) / 2.0;
    }
    /** @return Visual reprojection error scale for 2D residuals. */
    double errorMultiplier() const override {
        return error_multiplier_;
    }

    /** @return 0 (omni model has no single pinhole fx; vikit convention). */
    double fx() const override {
        return 0.0;
    }
    /** @return 0 (omni model has no single pinhole fy). */
    double fy() const override {
        return 0.0;
    }
    /** @return Principal point u from @ref OcamModel::yc. */
    double cx() const override {
        return ocam_.yc;
    }
    /** @return Principal point v from @ref OcamModel::xc. */
    double cy() const override {
        return ocam_.xc;
    }

    /** @return Underlying Scaramuzza calibration. */
    const OcamModel& model() const {
        return ocam_;
    }

private:
    double computeErrorMultiplier();
    bool LoadCalibFile(const std::string& path);

    OcamModel ocam_;
    double error_multiplier_ = 1.0;
    bool valid_ = false;
};

}  // namespace camera
}  // namespace argus

#endif  // ARGUS_COMMON_CAMERA_OMNI_CAMERA_HPP_
