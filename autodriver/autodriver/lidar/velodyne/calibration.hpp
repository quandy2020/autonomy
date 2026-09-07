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
 * @brief Velodyne beam calibration (vert_correction radians).
 */

#ifndef AUTODRIVER_LIDAR_VELODYNE_CALIBRATION_HPP_
#define AUTODRIVER_LIDAR_VELODYNE_CALIBRATION_HPP_

#include <string>
#include <vector>

namespace autodriver {
namespace lidar {
namespace velodyne {

struct BeamCalibration {
    std::vector<double> vert_correction_rad;  // indexed by laser_id
};

/**
 * @brief Loads lasers[].vert_correction (radians) from YAML.
 */
bool LoadBeamCalibrationYaml(const std::string& path, BeamCalibration* out,
                             std::string* error = nullptr);

/**
 * @brief Built-in VLP-16 vertical angles (degrees → radians).
 */
BeamCalibration DefaultVlp16Calibration();

}  // namespace velodyne
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_VELODYNE_CALIBRATION_HPP_
