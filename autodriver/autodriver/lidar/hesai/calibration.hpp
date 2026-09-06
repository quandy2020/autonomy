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
 * @brief Hesai / PandarXT beam elevation calibration.
 */

#ifndef AUTODRIVER_LIDAR_HESAI_CALIBRATION_HPP_
#define AUTODRIVER_LIDAR_HESAI_CALIBRATION_HPP_

#include <array>
#include <string>

#include "autodriver/lidar/hesai/packet.hpp"

namespace autodriver {
namespace lidar {
namespace hesai {

/**
 * @brief Per-channel vertical angles in degrees (channel index 0..31).
 * Same units as the XT32 manual table; convert applies DegToRad.
 */
struct BeamCalibration {
    std::array<double, kChannelsPerBlock> elev_deg{};
};

/**
 * @brief Built-in PandarXT / XT32 elevations (+15° … −16°).
 */
BeamCalibration DefaultXt32Calibration();

/**
 * @brief Load lasers[].vert_correction from YAML into degrees.
 *
 * Default unit is **degrees** (XT32 manual). Set root `unit: rad` or
 * `angles_in_radians: true` to interpret radians.
 */
bool LoadBeamCalibrationYaml(const std::string& path, BeamCalibration* out,
                             std::string* error = nullptr);

}  // namespace hesai
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_HESAI_CALIBRATION_HPP_
