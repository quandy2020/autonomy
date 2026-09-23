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
 * @brief Devernay–Faugeras FOV distortion model (Kalibr pinhole-fov).
 *
 * Distorts the incidence angle with scalar @f$ w @f$ before pinhole projection; @f$ w\to 0 @f$ degenerates to ideal pinhole.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_FOV_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_FOV_HPP_

#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

/**
 * @class autonomy::localization::atlas::sensor::camera::Fov
 * @brief FOV (field-of-view) distorted pinhole: radial distortion via @f$ w @f$ (radians).
 *
 * Matches Kalibr `pinhole-fov`. Normalized radius is mapped by
 * @f$ r_d = \tan(r w)/(2\tan(w/2)) @f$ then scaled by focal length to pixels;
 * `Unproject` applies the inverse. Assigns a global `id` on construction.
 *
 * @code{.cpp}
 * camera::Fov cam(fx, fy, cx, cy, 0.9);  // w
 * Vec2 uv = cam.Project(point_c);
 * @endcode
 */
class Fov : public GeometricCamera {
public:
    /**
     * @brief Default intrinsics (unit focal, zero distortion) and assign `id`.
     */
    Fov() { id = next_id++; }

    /**
     * @brief Specify pinhole intrinsics and FOV distortion.
     * @param fx Focal length fx (pixels).
     * @param fy Focal length fy (pixels).
     * @param cx Principal point cx.
     * @param cy Principal point cy.
     * @param w Distortion parameter (radians); near 0 ≈ undistorted.
     */
    Fov(double fx, double fy, double cx, double cy, double w)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), w_(w) {
        id = next_id++;
    }

    /**
     * @brief FOV-distort and project to pixels.
     * @param point_camera Camera-frame 3D point.
     * @return Pixel (u, v).
     */
    Vec2 Project(const Vec3& point_camera) const override;

    /**
     * @brief FOV unproject to a camera-frame point at the given depth.
     * @param pixel Pixel.
     * @param depth Depth scale (meters), default 1.0.
     * @return Camera-frame point.
     */
    Vec3 Unproject(const Vec2& pixel,
                                 double depth = 1.0) const override;

    /**
     * @brief Return `Type::kFov`.
     */
    Type type() const override { return Type::kFov; }

    /**
     * @brief Return `"fov"`.
     */
    const char* type_name() const override { return "fov"; }

    double fx() const override { return fx_; }
    double fy() const override { return fy_; }
    double cx() const override { return cx_; }
    double cy() const override { return cy_; }

    /**
     * @brief FOV distortion parameter @f$ w @f$ (radians).
     */
    double w() const { return w_; }

private:
    double fx_ = 1.0;  ///< Focal length fx.
    double fy_ = 1.0;  ///< Focal length fy.
    double cx_ = 0.0;  ///< Principal point cx.
    double cy_ = 0.0;  ///< Principal point cy.
    double w_ = 0.0;   ///< FOV distortion parameter w (radians).
};

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_FOV_HPP_
