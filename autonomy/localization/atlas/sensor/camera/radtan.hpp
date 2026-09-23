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
 * @brief Pinhole + Brown–Conrady distortion (OpenCV / Kalibr pinhole-radtan).
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_RADTAN_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_RADTAN_HPP_

#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

/**
 * @class autonomy::localization::atlas::sensor::camera::RadTan
 * @brief Perspective projection with radial–tangential distortion (k1,k2,p1,p2[,k3]).
 *
 * Parameter order matches OpenCV / Kalibr. `Unproject` solves via iterative
 * undistortion.
 *
 * @note Private `Distort` maps normalized-plane points to the distorted plane;
 *       used only by `Project`.
 */
class RadTan : public GeometricCamera {
public:
    /**
     * @brief Default zero-distortion pinhole and assign `id`.
     */
    RadTan() { id = next_id++; }

    /**
     * @brief Full radtan parameter constructor.
     * @param fx Focal length fx.
     * @param fy Focal length fy.
     * @param cx Principal point cx.
     * @param cy Principal point cy.
     * @param k1 Radial distortion k1.
     * @param k2 Radial distortion k2.
     * @param p1 Tangential distortion p1.
     * @param p2 Tangential distortion p2.
     * @param k3 Radial distortion k3, default 0.
     */
    RadTan(double fx, double fy, double cx, double cy, double k1, double k2,
           double p1, double p2, double k3 = 0.0)
        : fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_(cy),
          k1_(k1),
          k2_(k2),
          p1_(p1),
          p2_(p2),
          k3_(k3) {
        id = next_id++;
    }

    /**
     * @brief Distort and project to pixels.
     * @param point_camera Camera-frame point.
     * @return Pixel (u, v).
     */
    Vec2 Project(const Vec3& point_camera) const override;

    /**
     * @brief Undistort and unproject.
     * @param pixel Distorted pixel.
     * @param depth Depth (meters).
     * @return Camera-frame point.
     */
    Vec3 Unproject(const Vec2& pixel,
                                 double depth = 1.0) const override;

    Type type() const override { return Type::kRadTan; }
    const char* type_name() const override { return "radtan"; }
    double fx() const override { return fx_; }
    double fy() const override { return fy_; }
    double cx() const override { return cx_; }
    double cy() const override { return cy_; }

    /** @brief Radial distortion k1. */
    double k1() const { return k1_; }
    /** @brief Radial distortion k2. */
    double k2() const { return k2_; }
    /** @brief Tangential distortion p1. */
    double p1() const { return p1_; }
    /** @brief Tangential distortion p2. */
    double p2() const { return p2_; }
    /** @brief Radial distortion k3. */
    double k3() const { return k3_; }

private:
    /**
     * @brief Normalized plane (x,y) → distorted normalized coordinates.
     * @param x Undistorted normalized x.
     * @param y Undistorted normalized y.
     * @return Distorted (x', y').
     */
    Vec2 Distort(double x, double y) const;

    double fx_ = 1.0;  ///< Focal length fx.
    double fy_ = 1.0;  ///< Focal length fy.
    double cx_ = 0.0;  ///< Principal point cx.
    double cy_ = 0.0;  ///< Principal point cy.
    double k1_ = 0.0;  ///< Radial distortion k1.
    double k2_ = 0.0;  ///< Radial distortion k2.
    double p1_ = 0.0;  ///< Tangential distortion p1.
    double p2_ = 0.0;  ///< Tangential distortion p2.
    double k3_ = 0.0;  ///< Radial distortion k3.
};

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_RADTAN_HPP_
