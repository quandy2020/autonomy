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
 * @brief Kannala–Brandt equidistant fisheye (OpenCV fisheye / ORB-SLAM3 KB8).
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_KANNALA_BRANDT_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_KANNALA_BRANDT_HPP_

#include <vector>

#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

/**
 * @class autonomy::localization::atlas::sensor::camera::KannalaBrandt
 * @brief Equidistant fisheye projection (OpenCV fisheye / ORB KB8 / Kalibr equi).
 *
 * Parameter vector: `[fx, fy, cx, cy, k1, k2, k3, k4]`.
 *
 * @code{.cpp}
 * std::vector<double> p = {fx, fy, cx, cy, k1, k2, k3, k4};
 * camera::KannalaBrandt cam(p);
 * @endcode
 */
class KannalaBrandt : public GeometricCamera {
public:
    /**
     * @brief Default params (unit focal, zero distortion) and assign `id`.
     */
    KannalaBrandt();

    /**
     * @brief Construct from an 8-vector of parameters.
     * @param params Length ≥ 8: fx,fy,cx,cy,k1,k2,k3,k4; pad defaults if shorter.
     */
    explicit KannalaBrandt(const std::vector<double>& params);

    /**
     * @brief Explicit intrinsics and distortion constructor.
     * @param fx Focal length fx.
     * @param fy Focal length fy.
     * @param cx Principal point cx.
     * @param cy Principal point cy.
     * @param k1 Fisheye distortion k1.
     * @param k2 Fisheye distortion k2.
     * @param k3 Fisheye distortion k3.
     * @param k4 Fisheye distortion k4.
     */
    KannalaBrandt(double fx, double fy, double cx, double cy, double k1,
                  double k2, double k3, double k4);

    /**
     * @brief Fisheye project to pixels.
     * @param point_camera Camera-frame point.
     * @return Pixel (u, v).
     */
    Vec2 Project(const Vec3& point_camera) const override;

    /**
     * @brief Fisheye unproject (iterative angle solve).
     * @param pixel Pixel.
     * @param depth Depth scale (meters).
     * @return Camera-frame point.
     */
    Vec3 Unproject(const Vec2& pixel,
                                 double depth = 1.0) const override;

    Type type() const override { return Type::kKannalaBrandt; }
    const char* type_name() const override {
        return "kannala_brandt";
    }
    double fx() const override { return params_[0]; }
    double fy() const override { return params_[1]; }
    double cx() const override { return params_[2]; }
    double cy() const override { return params_[3]; }

    /**
     * @brief Read-only parameter vector `[fx,fy,cx,cy,k1,k2,k3,k4]`.
     * @return Reference to internal parameters.
     */
    const std::vector<double>& params() const { return params_; }

private:
    std::vector<double> params_;  ///< fx,fy,cx,cy,k1,k2,k3,k4.
};

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_KANNALA_BRANDT_HPP_
