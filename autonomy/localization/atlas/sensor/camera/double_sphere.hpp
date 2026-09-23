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
 * @brief Usenko double-sphere camera model (Kalibr / Basalt `ds`).
 *
 * Two spherical projections then perspective; project/unproject have closed or stable numeric forms; suits wide-angle.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_DOUBLE_SPHERE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_DOUBLE_SPHERE_HPP_

#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

/**
 * @class autonomy::localization::atlas::sensor::camera::DoubleSphere
 * @brief Double-sphere model: parameters @f$ (f_x,f_y,c_x,c_y,\xi,\alpha) @f$.
 *
 * Matches Kalibr/Basalt `double_sphere` / `ds`. @f$ \xi @f$ controls first-sphere shift;
 * @f$ \alpha @f$ blends the second-sphere projection denominator; together they set FOV and distortion shape.
 * Assigns a global `id` on construction.
 *
 * @code{.cpp}
 * camera::DoubleSphere cam(fx, fy, cx, cy, -0.2, 0.6);  // xi, alpha
 * Vec2 uv = cam.Project(point_c);
 * @endcode
 */
class DoubleSphere : public GeometricCamera {
public:
    /**
     * @brief Default intrinsics (ξ=0, α=0.5) and assign `id`.
     */
    DoubleSphere() { id = next_id++; }

    /**
     * @brief Specify intrinsics and double-sphere parameters.
     * @param fx Focal length fx (pixels).
     * @param fy Focal length fy (pixels).
     * @param cx Principal point cx.
     * @param cy Principal point cy.
     * @param xi First-sphere shift @f$ \xi @f$.
     * @param alpha Second-sphere blend @f$ \alpha @f$.
     */
    DoubleSphere(double fx, double fy, double cx, double cy, double xi,
                 double alpha)
        : fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_(cy),
          xi_(xi),
          alpha_(alpha) {
        id = next_id++;
    }

    /**
     * @brief Double-sphere project to pixels.
     * @param point_camera Camera-frame 3D point.
     * @return Pixel (u, v).
     */
    Vec2 Project(const Vec3& point_camera) const override;

    /**
     * @brief Double-sphere unproject to a camera-frame point at depth.
     * @param pixel Pixel.
     * @param depth Depth scale (meters), default 1.0.
     * @return Camera-frame point.
     */
    Vec3 Unproject(const Vec2& pixel,
                                 double depth = 1.0) const override;

    /**
     * @brief Return `Type::kDoubleSphere`.
     */
    Type type() const override { return Type::kDoubleSphere; }

    /**
     * @brief Return `"double_sphere"`.
     */
    const char* type_name() const override {
        return "double_sphere";
    }

    double fx() const override { return fx_; }
    double fy() const override { return fy_; }
    double cx() const override { return cx_; }
    double cy() const override { return cy_; }

    /**
     * @brief First-sphere shift @f$ \xi @f$.
     */
    double xi() const { return xi_; }

    /**
     * @brief Second-sphere blend @f$ \alpha @f$.
     */
    double alpha() const { return alpha_; }

private:
    double fx_ = 1.0;     ///< Focal length fx.
    double fy_ = 1.0;     ///< Focal length fy.
    double cx_ = 0.0;     ///< Principal point cx.
    double cy_ = 0.0;     ///< Principal point cy.
    double xi_ = 0.0;     ///< Double-sphere ξ.
    double alpha_ = 0.5;  ///< Double-sphere α.
};

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_DOUBLE_SPHERE_HPP_
