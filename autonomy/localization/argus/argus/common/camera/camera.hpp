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
 * @file camera.hpp
 * @brief Umbrella header for rpg_vikit-compatible Argus camera models.
 *
 * Includes (in dependency order):
 * - @ref argus::camera::AbstractCamera — base projection API
 * - Pinhole, Equidistant, ATAN (FOV), Polynomial, Omni concrete models
 * - @ref argus::camera::CameraIntrinsics and YAML/OCam loaders
 *
 * Models: Pinhole, Equidistant, ATAN (FOV), Polynomial, Omni.
 *
 * This header aggregates includes only; see each model header for API details.
 * @see https://github.com/xuankuzcr/rpg_vikit
 */

#ifndef ARGUS_COMMON_CAMERA_CAMERA_HPP_
#define ARGUS_COMMON_CAMERA_CAMERA_HPP_

#include "argus/common/camera/abstract_camera.hpp" /**< Base camera interface. */
#include "argus/common/camera/atan_camera.hpp"     /**< ATAN / FOV model. */
#include "argus/common/camera/camera_loader.hpp" /**< YAML / intrinsics loading. */
#include "argus/common/camera/equidistant_camera.hpp" /**< Kannala–Brandt fisheye. */
#include "argus/common/camera/omni_camera.hpp" /**< Scaramuzza omnidirectional. */
#include "argus/common/camera/pinhole_camera.hpp"    /**< Pinhole + radtan. */
#include "argus/common/camera/polynomial_camera.hpp" /**< Polynomial distortion. */

#endif  // ARGUS_COMMON_CAMERA_CAMERA_HPP_
