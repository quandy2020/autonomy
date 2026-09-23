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
 * @brief Fitzgibbon radial division distortion (OpenVSLAM RadialDivision).
 *
 * Approximates wide-angle radial distortion with scalar @f$ k @f$ division; more compact than polynomial radtan.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_RADIAL_DIVISION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_RADIAL_DIVISION_HPP_

#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

/**
 * @class autonomy::localization::atlas::sensor::camera::RadialDivision
 * @brief Division model: normalized plane @f$ x'=x/(1+k r^2),\ y'=y/(1+k r^2) @f$.
 *
 * Factory names `radial_division` / `division`. Pinhole-normalize, then divide by radius-squared,
 * then multiply by focal length; `Unproject` solves undistorted coords analytically or iteratively. @f$ k=0 @f$ degenerates to
 * pinhole. Assigns a global `id` on construction.
 *
 * @code{.cpp}
 * camera::RadialDivision cam(fx, fy, cx, cy, -0.1);  // k
 * Vec2 uv = cam.Project(point_c);
 * @endcode
 */
class RadialDivision : public GeometricCamera {
public:
    /**
     * @brief Default intrinsics (unit focal, k=0) and assign `id`.
     */
    RadialDivision() { id = next_id++; }

    /**
     * @brief Specify pinhole intrinsics and division distortion.
     * @param fx Focal length fx (pixels).
     * @param fy Focal length fy (pixels).
     * @param cx Principal point cx.
     * @param cy Principal point cy.
     * @param k Division distortion coeff; negative often barrel.
     */
    RadialDivision(double fx, double fy, double cx, double cy, double k)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), k_(k) {
        id = next_id++;
    }

    /**
     * @brief Division-model project to pixels.
     * @param point_camera Camera-frame 3D point.
     * @return Pixel (u, v).
     */
    Vec2 Project(const Vec3& point_camera) const override;

    /**
     * @brief Undistort-unproject to a camera-frame point at depth.
     * @param pixel Pixel.
     * @param depth Depth scale (meters), default 1.0.
     * @return Camera-frame point.
     */
    Vec3 Unproject(const Vec2& pixel,
                                 double depth = 1.0) const override;

    /**
     * @brief Return `Type::kRadialDivision`.
     */
    Type type() const override { return Type::kRadialDivision; }

    /**
     * @brief Return `"radial_division"`.
     */
    const char* type_name() const override {
        return "radial_division";
    }

    double fx() const override { return fx_; }
    double fy() const override { return fy_; }
    double cx() const override { return cx_; }
    double cy() const override { return cy_; }

    /**
     * @brief Division distortion coefficient @f$ k @f$.
     */
    double k() const { return k_; }

private:
    double fx_ = 1.0;  ///< Focal length fx.
    double fy_ = 1.0;  ///< Focal length fy.
    double cx_ = 0.0;  ///< Principal point cx.
    double cy_ = 0.0;  ///< Principal point cy.
    double k_ = 0.0;   ///< Division distortion coefficient k.
};

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_RADIAL_DIVISION_HPP_
