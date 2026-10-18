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
 * @file camera_loader.hpp
 * @brief Load Argus camera models from YAML / OCam file (rpg_vikit-compatible).
 *
 * Supported `cam_model` (aliases in parentheses):
 *   - Pinhole
 *   - EquidistantCamera (Equidistant)
 *   - ATANCamera (ATAN, FOV)
 *   - PolynomialCamera (Polynomial)
 *   - OmniCamera (Omni) — uses `ocam_calib_file` Scaramuzza text
 */

#ifndef ARGUS_COMMON_CAMERA_CAMERA_LOADER_HPP_
#define ARGUS_COMMON_CAMERA_CAMERA_LOADER_HPP_

#include <memory>
#include <string>

#include "argus/common/camera/abstract_camera.hpp"

namespace argus {
namespace camera {

/** @brief Parsed camera parameters before model construction. */
struct CameraIntrinsics {
    /** @brief Model name (`Pinhole`, `EquidistantCamera`, …). */
    std::string model;
    /** @brief Image width in pixels. */
    int width = 0;
    /** @brief Image height in pixels. */
    int height = 0;
    /** @brief Image pyramid scale factor (vikit `scale`). */
    double scale = 1.0;
    /** @brief Focal length along u (pixels). */
    double fx = 0.0;
    /** @brief Focal length along v (pixels). */
    double fy = 0.0;
    /** @brief Principal point u (pixels). */
    double cx = 0.0;
    /** @brief Principal point v (pixels). */
    double cy = 0.0;
    /** @brief OpenCV radtan k1 (Pinhole). */
    double d0 = 0.0;
    /** @brief OpenCV radtan k2. */
    double d1 = 0.0;
    /** @brief OpenCV radtan p1. */
    double d2 = 0.0;
    /** @brief OpenCV radtan p2. */
    double d3 = 0.0;
    /** @brief OpenCV radtan k3. */
    double d4 = 0.0;
    /** @brief Equidistant distortion k1 / polynomial k2 / shared higher-order
     * term. */
    double k1 = 0.0;
    /** @brief Equidistant k2 / polynomial k3. */
    double k2 = 0.0;
    /** @brief Equidistant k3 / polynomial k4. */
    double k3 = 0.0;
    /** @brief Equidistant k4 / polynomial k5. */
    double k4 = 0.0;
    /** @brief Polynomial distortion k5. */
    double k5 = 0.0;
    /** @brief Polynomial distortion k6. */
    double k6 = 0.0;
    /** @brief Polynomial distortion k7. */
    double k7 = 0.0;
    /** @brief ATAN FOV parameter (also accepted as `cam_s`). */
    double s = 0.0;
    /** @brief Polynomial skew. */
    double skew = 0.0;
    /**
     * @brief If true, ATAN fx/fy/cx/cy are fractions of W/H (rpg_vikit).
     *        Default false: absolute pixels like Pinhole.
     */
    bool atan_normalized = false;
    /** @brief Scaramuzza calib path for OmniCamera. */
    std::string ocam_calib_file;
};

/**
 * @brief Construct a camera from explicit intrinsics.
 * @param intrinsics Model name and calibration coefficients.
 * @return Owning pointer to the model, or nullptr if the model is unknown or
 * invalid.
 */
std::unique_ptr<AbstractCamera> Create(const CameraIntrinsics& intrinsics);

/**
 * @brief Create from FAST-LIVO2 / vikit-style YAML (`cam_model`, `cam_width`,
 * …).
 * @param yaml_path Path to the calibration YAML file.
 * @return Owning pointer to the model, or nullptr on parse/validation failure.
 */
std::unique_ptr<AbstractCamera> LoadFromYaml(const std::string& yaml_path);

/**
 * @brief Load into a raw pointer (caller owns with `delete`).
 * @param yaml_path Path to the calibration YAML file.
 * @param[out] cam Receives `new`-allocated camera on success; unchanged on
 * failure.
 * @return True if loading and model construction succeeded.
 */
bool LoadFromYaml(const std::string& yaml_path, AbstractCamera*& cam);

/**
 * @brief Construct a camera from intrinsics into a raw pointer (caller owns
 * with `delete`).
 * @param intrinsics Model name and calibration coefficients.
 * @param[out] cam Receives `new`-allocated camera on success; unchanged on
 * failure.
 * @return True if model construction succeeded.
 */
bool Create(const CameraIntrinsics& intrinsics, AbstractCamera*& cam);

}  // namespace camera
}  // namespace argus

#endif  // ARGUS_COMMON_CAMERA_CAMERA_LOADER_HPP_
