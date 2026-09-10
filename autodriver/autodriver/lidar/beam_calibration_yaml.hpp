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
 * @brief Shared YAML loader for lasers[].vert_correction (Velodyne / Hesai).
 */

#ifndef AUTODRIVER_LIDAR_BEAM_CALIBRATION_YAML_HPP_
#define AUTODRIVER_LIDAR_BEAM_CALIBRATION_YAML_HPP_

#include <map>
#include <string>

namespace autodriver {
namespace lidar {

/**
 * @brief Result of loading a beam-calibration YAML file.
 */
struct LaserVertCorrectionTable {
    /** laser_id → vert_correction value (units as stored in the file). */
    std::map<int, double> by_id;
    /**
     * @brief True when root `unit: rad` / `angles_in_radians: true`.
     * Velodyne files are conventionally radians; Hesai defaults to degrees.
     */
    bool angles_in_radians = false;
};

/**
 * @brief Load lasers[].laser_id + vert_correction from a YAML file.
 * @param path Path to the calibration YAML.
 * @param[out] out Filled table on success; must be non-null.
 * @param[out] error Optional human-readable failure reason.
 * @return true when at least one laser entry was parsed.
 *
 * Accepts either a root `lasers:` sequence or a top-level sequence.
 */
bool LoadLaserVertCorrectionsYaml(const std::string& path,
                                  LaserVertCorrectionTable* out,
                                  std::string* error = nullptr);

}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_BEAM_CALIBRATION_YAML_HPP_
