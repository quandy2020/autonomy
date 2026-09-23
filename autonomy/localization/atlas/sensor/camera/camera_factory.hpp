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
 * @brief Factory to build `GeometricCamera` from model name + intrinsics / YAML.
 *
 * Matches Kalibr / ORB camera-type strings; unknown models may make `Create` throw
 * or fall back per implementation (see `.cpp`).
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_CAMERA_FACTORY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_CAMERA_FACTORY_HPP_

#include <memory>
#include <string>

#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace YAML {
class Node;
}  // namespace YAML

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {
namespace camera {

/**
 * @class autonomy::localization::atlas::sensor::camera::CameraFactory
 * @brief Build a concrete `GeometricCamera` from model name and intrinsics.
 *
 * Recognized model names (aliases in parentheses):
 * - `pinhole`
 * - `radtan`(opencv, brown, perspective)
 * - `kannala_brandt`(kannala, fisheye, equi, kb8)
 * - `fov`
 * - `ucm`(mei, omni)
 * - `eucm`
 * - `double_sphere`(ds)
 * - `equirectangular`(equirect, panorama)
 * - `radial_division`(division)
 *
 * @code{.cpp}
 * CameraFactory::Intrinsics i;
 * i.model = "radtan";
 * i.fx = 500; i.fy = 500; i.cx = 320; i.cy = 240;
 * i.k1 = -0.1; i.k2 = 0.01; i.p1 = 0; i.p2 = 0;
 * auto cam = CameraFactory::Create(i);
 * @endcode
 */
class CameraFactory {
public:
    /**
     * @struct Intrinsics
     * @brief Unified intrinsics / distortion bag; unused fields may stay 0.
     */
    struct Intrinsics {
        std::string model = "pinhole";  ///< Model name (case-insensitive; see alias table).
        double fx = 1.0;  ///< Focal length fx.
        double fy = 1.0;  ///< Focal length fy.
        double cx = 0.0;  ///< Principal point cx.
        double cy = 0.0;  ///< Principal point cy.
        // Distortion / model-specific (unused slots ignored).
        double k1 = 0.0;  ///< Radial / KB / division-model k1.
        double k2 = 0.0;  ///< k2.
        double k3 = 0.0;  ///< k3(radtan / KB).
        double k4 = 0.0;  ///< k4(KB).
        double p1 = 0.0;  ///< Tangential distortion p1.
        double p2 = 0.0;  ///< Tangential distortion p2.
        double w = 0.0;      ///< FOV distortion parameter w (radians).
        double xi = 0.0;     ///< ξ for UCM / DoubleSphere.
        double alpha = 0.5;  ///< α for EUCM / DoubleSphere.
        double beta = 1.0;   ///< β for EUCM.
        double k = 0.0;      ///< k for RadialDivision.
        int width = 0;   ///< Image width; 0=unknown.
        int height = 0;  ///< Image height; 0=unknown.
    };

    /**
     * @brief Create a camera instance from `Intrinsics`.
     * @param intrinsics Model name and parameters.
     * @return Owned camera; impl may throw `std::invalid_argument` on failure.
     */
    static std::unique_ptr<GeometricCamera> Create(
        const Intrinsics& intrinsics);

    /**
     * @brief Create camera from a YAML node (reads model / fx / fy / …).
     * @param node YAML node with camera fields.
     * @return Owned camera; missing keys use defaults; invalid model may throw.
     */
    static std::unique_ptr<GeometricCamera> CreateFromYaml(
        const YAML::Node& node);

    /**
     * @brief Parse model name string to enum.
     * @param name Model name or alias (case-insensitive).
     * @return Matching `GeometricCamera::Type`; usually falls back to `kPinhole` if unknown.
     */
    static GeometricCamera::Type ParseModelType(
        const std::string& name);
};

}  // namespace camera
}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_CAMERA_FACTORY_HPP_
