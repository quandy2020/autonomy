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
 * @brief Camera module umbrella header: all GeometricCamera models + ORB-SLAM3 aliases.
 *
 * Callers may `#include "…/sensor/camera.hpp"` only; prefer
 * `camera::Pinhole` / `camera::KannalaBrandt` in new code — aliases are for compat.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_HPP_

#include "autonomy/localization/atlas/sensor/camera/camera_factory.hpp"
#include "autonomy/localization/atlas/sensor/camera/double_sphere.hpp"
#include "autonomy/localization/atlas/sensor/camera/equirectangular.hpp"
#include "autonomy/localization/atlas/sensor/camera/eucm.hpp"
#include "autonomy/localization/atlas/sensor/camera/fov.hpp"
#include "autonomy/localization/atlas/sensor/camera/kannala_brandt.hpp"
#include "autonomy/localization/atlas/sensor/camera/pinhole.hpp"
#include "autonomy/localization/atlas/sensor/camera/radial_division.hpp"
#include "autonomy/localization/atlas/sensor/camera/radtan.hpp"
#include "autonomy/localization/atlas/sensor/camera/ucm.hpp"
#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace sensor {

using PinholeCamera = camera::Pinhole;  ///< ORB-SLAM3-style alias → `camera::Pinhole`.
using KannalaBrandt8 = camera::KannalaBrandt;  ///< ORB-SLAM3 KB8 alias.

}  // namespace sensor
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_CAMERA_HPP_
