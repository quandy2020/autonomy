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
 * @brief Mei unified / omnidirectional camera model UCM (Kalibr omni).
 *
 * Projects the unit sphere onto a plane offset by @f$ \xi @f$ from the center; suits wide-angle and catadioptric lenses.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_UCM_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_UCM_HPP_

#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

/**
 * @class autonomy::localization::atlas::sensor::camera::Ucm
 * @brief Mei Unified Camera Model:@f$ u=f_x\,x/(\xi\lVert p\rVert+z)+c_x @f$.
 *
 * Matches Kalibr `omni` / `ucm`. Parameter @f$ \xi\in[0,1] @f$: @f$ \xi=0 @f$ degenerates to pinhole;
 * larger @f$ \xi @f$ covers a wider FOV. `Unproject` recovers spherical direction then scales by depth.
 *
 * @code{.cpp}
 * camera::Ucm cam(fx, fy, cx, cy, 0.5);  // xi
 * Vec2 uv = cam.Project(point_c);
 * @endcode
 */
class Ucm : public GeometricCamera {
public:
    /**
     * @brief Default intrinsics (unit focal, @f$ \xi=0 @f$) and assign `id`.
     */
    Ucm() { id = next_id++; }

    /**
     * @brief Specify intrinsics and mirror parameter.
     * @param fx Focal length fx (pixels).
     * @param fy Focal length fy (pixels).
     * @param cx Principal point cx.
     * @param cy Principal point cy.
     * @param xi Unified-model parameter @f$ \xi @f$ (usually ∈ [0,1]).
     */
    Ucm(double fx, double fy, double cx, double cy, double xi)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), xi_(xi) {
        id = next_id++;
    }

    /**
     * @brief UCM project to pixels.
     * @param point_camera Camera-frame 3D point.
     * @return Pixel (u, v).
     */
    Vec2 Project(const Vec3& point_camera) const override;

    /**
     * @brief UCM unproject to a camera-frame point at the given depth.
     * @param pixel Pixel.
     * @param depth Depth scale (meters), default 1.0.
     * @return Camera-frame point.
     */
    Vec3 Unproject(const Vec2& pixel,
                                 double depth = 1.0) const override;

    /**
     * @brief Return `Type::kUcm`.
     */
    Type type() const override { return Type::kUcm; }

    /**
     * @brief Return `"ucm"`.
     */
    const char* type_name() const override { return "ucm"; }

    double fx() const override { return fx_; }
    double fy() const override { return fy_; }
    double cx() const override { return cx_; }
    double cy() const override { return cy_; }

    /**
     * @brief Unified-model parameter @f$ \xi @f$.
     */
    double xi() const { return xi_; }

private:
    double fx_ = 1.0;  ///< Focal length fx.
    double fy_ = 1.0;  ///< Focal length fy.
    double cx_ = 0.0;  ///< Principal point cx.
    double cy_ = 0.0;  ///< Principal point cy.
    double xi_ = 0.0;  ///< Mei mirror parameter ξ.
};

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_UCM_HPP_
