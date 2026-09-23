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
 * @brief Implementations of GeometricCamera::next_id and GeometricCameraTypeName.
 */

#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {

long unsigned int GeometricCamera::next_id = 0;

const char* GeometricCameraTypeName(GeometricCamera::Type type) {
    switch (type) {
        case GeometricCamera::Type::kPinhole:
            return "pinhole";
        case GeometricCamera::Type::kRadTan:
            return "radtan";
        case GeometricCamera::Type::kKannalaBrandt:
            return "kannala_brandt";
        case GeometricCamera::Type::kFov:
            return "fov";
        case GeometricCamera::Type::kUcm:
            return "ucm";
        case GeometricCamera::Type::kEucm:
            return "eucm";
        case GeometricCamera::Type::kDoubleSphere:
            return "double_sphere";
        case GeometricCamera::Type::kEquirectangular:
            return "equirectangular";
        case GeometricCamera::Type::kRadialDivision:
            return "radial_division";
    }
    return "unknown";
}

}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
