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
 * @brief Ideal pinhole camera model (ORB-SLAM3 Pinhole / Kalibr pinhole-none).
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_PINHOLE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_PINHOLE_HPP_

#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

/**
 * @class autonomy::localization::atlas::sensor::camera::Pinhole
 * @brief Undistorted perspective projection:
 *        @f$ u=f_x x/z+c_x,\ v=f_y y/z+c_y @f$.
 *
 * Matches ORB-SLAM3 `Pinhole` and Kalibr `pinhole-none`. Assigns a global `id`
 * on construction.
 *
 * @code{.cpp}
 * camera::Pinhole cam(fx, fy, cx, cy);
 * Vec2 uv = cam.Project(Vec3(0.1, 0.0, 2.0));
 * @endcode
 */
class Pinhole : public GeometricCamera {
public:
    /**
     * @brief Default intrinsics (fx=fy=1, cx=cy=0) and assign `id`.
     */
    Pinhole() { id = next_id++; }

    /**
     * @brief Construct with pinhole intrinsics.
     * @param fx Focal length fx (pixels).
     * @param fy Focal length fy (pixels).
     * @param cx Principal point cx.
     * @param cy Principal point cy.
     */
    Pinhole(double fx, double fy, double cx, double cy)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {
        id = next_id++;
    }

    /**
     * @brief Project a camera-frame point to pixels.
     * @param point_camera Camera-frame 3D point; z should be > 0.
     * @return Pixel (u, v).
     */
    Vec2 Project(const Vec3& point_camera) const override;

    /**
     * @brief Unproject a pixel to a camera-frame point at the given depth.
     * @param pixel Pixel.
     * @param depth Depth z (meters), default 1.0.
     * @return Camera-frame point.
     */
    Vec3 Unproject(const Vec2& pixel,
                                 double depth = 1.0) const override;

    /**
     * @brief Return `Type::kPinhole`.
     */
    Type type() const override { return Type::kPinhole; }

    /**
     * @brief Return `"pinhole"`.
     */
    const char* type_name() const override { return "pinhole"; }

    double fx() const override { return fx_; }
    double fy() const override { return fy_; }
    double cx() const override { return cx_; }
    double cy() const override { return cy_; }

private:
    double fx_ = 1.0;  ///< Focal length fx.
    double fy_ = 1.0;  ///< Focal length fy.
    double cx_ = 0.0;  ///< Principal point cx.
    double cy_ = 0.0;  ///< Principal point cy.
};

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_PINHOLE_HPP_
