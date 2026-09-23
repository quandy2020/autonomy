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
 * @brief Extended Unified Camera Model (EUCM, Kalibr / Basalt).
 *
 * Extends UCM with @f$ \alpha,\beta @f$ for more flexible wide-angle fitting.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_EUCM_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_EUCM_HPP_

#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

/**
 * @class autonomy::localization::atlas::sensor::camera::Eucm
 * @brief Extended Unified Camera Model: @f$ d=\sqrt{\beta(x^2+y^2)+z^2} @f$,
 *        @f$ u=f_x\,x/(\alpha d+(1-\alpha)z)+c_x @f$.
 *
 * Matches Kalibr/Basalt `eucm`. @f$ \alpha @f$ blends sphere and perspective denominators; @f$ \beta @f$ scales
 * the lateral metric; suitable values cover >180° FOV. Assigns a global `id` on construction.
 *
 * @code{.cpp}
 * camera::Eucm cam(fx, fy, cx, cy, 0.6, 1.0);  // alpha, beta
 * Vec2 uv = cam.Project(point_c);
 * @endcode
 */
class Eucm : public GeometricCamera {
public:
    /**
     * @brief Default intrinsics (α=0.5, β=1) and assign `id`.
     */
    Eucm() { id = next_id++; }

    /**
     * @brief Specify intrinsics and EUCM parameters.
     * @param fx Focal length fx (pixels).
     * @param fy Focal length fy (pixels).
     * @param cx Principal point cx.
     * @param cy Principal point cy.
     * @param alpha Blend parameter @f$ \alpha @f$ (usually ∈ (0,1]).
     * @param beta Lateral scale @f$ \beta @f$ (usually > 0).
     */
    Eucm(double fx, double fy, double cx, double cy, double alpha, double beta)
        : fx_(fx),
          fy_(fy),
          cx_(cx),
          cy_(cy),
          alpha_(alpha),
          beta_(beta) {
        id = next_id++;
    }

    /**
     * @brief EUCM project to pixels.
     * @param point_camera Camera-frame 3D point.
     * @return Pixel (u, v).
     */
    Vec2 Project(const Vec3& point_camera) const override;

    /**
     * @brief EUCM unproject to a camera-frame point at the given depth.
     * @param pixel Pixel.
     * @param depth Depth scale (meters), default 1.0.
     * @return Camera-frame point.
     */
    Vec3 Unproject(const Vec2& pixel,
                                 double depth = 1.0) const override;

    /**
     * @brief Return `Type::kEucm`.
     */
    Type type() const override { return Type::kEucm; }

    /**
     * @brief Return `"eucm"`.
     */
    const char* type_name() const override { return "eucm"; }

    double fx() const override { return fx_; }
    double fy() const override { return fy_; }
    double cx() const override { return cx_; }
    double cy() const override { return cy_; }

    /**
     * @brief Blend parameter @f$ \alpha @f$.
     */
    double alpha() const { return alpha_; }

    /**
     * @brief Lateral scale @f$ \beta @f$.
     */
    double beta() const { return beta_; }

private:
    double fx_ = 1.0;     ///< Focal length fx.
    double fy_ = 1.0;     ///< Focal length fy.
    double cx_ = 0.0;     ///< Principal point cx.
    double cy_ = 0.0;     ///< Principal point cy.
    double alpha_ = 0.5;  ///< EUCM blend parameter α.
    double beta_ = 1.0;   ///< EUCM lateral parameter β.
};

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_EUCM_HPP_
