/*
 * Copyright 2026 Autodriver contributors
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
 * @brief Load static sensor extrinsics from YAML.
 */

#ifndef AUTODRIVER_COMMON_CALIBRATION_HPP_
#define AUTODRIVER_COMMON_CALIBRATION_HPP_

#include <string>

#include <Eigen/Geometry>

namespace autodriver {
namespace common {

/**
 * @brief Static transform parent←child (world/novatel ← lidar/camera).
 */
struct Extrinsic {
    std::string parent_frame;
    std::string child_frame;
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
};

/**
 * @brief Parses YAML with header.frame_id, child_frame_id, transform.{translation,rotation}.
 * Matches common velodyne params extrinsics YAML shape.
 */
bool LoadExtrinsicYaml(const std::string& path, Extrinsic* out,
                       std::string* error = nullptr);

}  // namespace common
}  // namespace autodriver

#endif  // AUTODRIVER_COMMON_CALIBRATION_HPP_
