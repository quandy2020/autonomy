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
 * @brief Full-sphere equirectangular (panoramic) projection model.
 *
 * Linearly maps azimuth and elevation to pixels, covering the full 360°×180° sphere.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_EQUIRECTANGULAR_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_EQUIRECTANGULAR_HPP_

#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

/**
 * @class autonomy::localization::atlas::sensor::camera::Equirectangular
 * @brief Equirectangular projection: @f$ u=f_x\arctan2(x,z)+c_x @f$,
 *        @f$ v=f_y\arcsin(y/\lVert p\rVert)+c_y @f$.
 *
 * Factory names `equirectangular` / `equirect` / `panorama`.
 * Convention: @f$ f_x\approx W/(2\pi),\ f_y\approx H/\pi,\ c_x\approx W/2,\ c_y\approx H/2 @f$.
 * Optional image size is written to base `width_`/`height_`; assigns a global `id` on construction.
 *
 * @code{.cpp}
 * camera::Equirectangular cam(W/(2*M_PI), H/M_PI, W/2.0, H/2.0, W, H);
 * Vec2 uv = cam.Project(point_c);
 * @endcode
 */
class Equirectangular : public GeometricCamera {
public:
    /**
     * @brief Default intrinsics and assign `id`.
     */
    Equirectangular() { id = next_id++; }

    /**
     * @brief Specify angle-to-pixel scales and optional image size.
     * @param fx Azimuth-to-pixel scale (usually ≈ width/(2π)).
     * @param fy Elevation-to-pixel scale (usually ≈ height/π).
     * @param cx Principal point cx (usually ≈ width/2).
     * @param cy Principal point cy (usually ≈ height/2).
     * @param width Image width; 0 means unknown.
     * @param height Image height; 0 means unknown.
     */
    Equirectangular(double fx, double fy, double cx, double cy, int width = 0,
                    int height = 0)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {
        width_ = width;
        height_ = height;
        id = next_id++;
    }

    /**
     * @brief Project a spherical direction to equirectangular pixels.
     * @param point_camera Camera-frame 3D point (any non-zero direction).
     * @return Pixel (u, v).
     */
    Vec2 Project(const Vec3& point_camera) const override;

    /**
     * @brief Recover spherical direction from pixel and scale to depth.
     * @param pixel Pixel.
     * @param depth Depth along ray (meters), default 1.0.
     * @return Camera-frame point.
     */
    Vec3 Unproject(const Vec2& pixel,
                                 double depth = 1.0) const override;

    /**
     * @brief Return `Type::kEquirectangular`.
     */
    Type type() const override { return Type::kEquirectangular; }

    /**
     * @brief Return `"equirectangular"`.
     */
    const char* type_name() const override {
        return "equirectangular";
    }

    double fx() const override { return fx_; }
    double fy() const override { return fy_; }
    double cx() const override { return cx_; }
    double cy() const override { return cy_; }

private:
    double fx_ = 1.0;  ///< Azimuth focal length (pixels/radian).
    double fy_ = 1.0;  ///< Elevation focal length (pixels/radian).
    double cx_ = 0.0;  ///< Principal point cx.
    double cy_ = 0.0;  ///< Principal point cy.
};

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_EQUIRECTANGULAR_HPP_
