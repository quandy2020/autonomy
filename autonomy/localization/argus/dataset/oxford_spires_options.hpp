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
 * @file oxford_spires_options.hpp
 * @brief Reader options for Oxford Spires Dataset sequences.
 */

#ifndef ARGUS_APPS_DATASETS_OXFORD_SPIRES_OPTIONS_HPP_
#define ARGUS_APPS_DATASETS_OXFORD_SPIRES_OPTIONS_HPP_

#include <string>

namespace argus {
namespace datasets {

/** @brief Reader options (camera choice, undistorted clouds, sync threshold). */
struct OxfordSpiresOptions {
    OxfordSpiresOptions();

    /** @brief cam0 (forward) / cam1 (left) / cam2 (right). */
    std::string camera = "cam0";
    /** @brief Prefer processed/vilens-slam/undist-clouds when present. */
    bool prefer_undistorted = false;
    /** @brief Max |t_img - t_lidar| for association (sec). */
    double max_image_dt_sec = 0.05;
    /** @brief Optional path to dataset-level calibration/ directory. */
    std::string calibration_dir;
};

}  // namespace datasets
}  // namespace argus

#endif  // ARGUS_APPS_DATASETS_OXFORD_SPIRES_OPTIONS_HPP_
