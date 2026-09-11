/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file calibration.cpp
 * @brief Hesai / PandarXT beam elevation calibration (implementation).
 */

#include "autodriver/lidar/hesai/calibration.hpp"

#include "autodriver/lidar/beam_calibration_yaml.hpp"

namespace autodriver {
namespace lidar {
namespace hesai {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

}  // namespace

BeamCalibration DefaultXt32Calibration() {
    BeamCalibration cal;
    // Manual Appendix I: channel 1 (top) = +15° … channel 32 (bottom) = -16°.
    cal.elev_deg = {{15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
                     -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13,
                     -14, -15, -16}};
    return cal;
}

bool LoadBeamCalibrationYaml(const std::string& path, BeamCalibration* out,
                             std::string* error) {
    if (out == nullptr) {
        if (error != nullptr) {
            *error = "null BeamCalibration";
        }
        return false;
    }
    LaserVertCorrectionTable table;
    if (!LoadLaserVertCorrectionsYaml(path, &table, error)) {
        return false;
    }
    std::size_t valid = 0;
    for (const auto& entry : table.by_id) {
        if (entry.first >= 0 &&
            static_cast<std::size_t>(entry.first) < kChannelsPerBlock) {
            ++valid;
        }
    }
    if (valid < kChannelsPerBlock) {
        if (error != nullptr) {
            *error = "need 32 laser vert_correction entries for XT32";
        }
        return false;
    }
    *out = DefaultXt32Calibration();
    for (const auto& entry : table.by_id) {
        if (entry.first < 0 ||
            static_cast<std::size_t>(entry.first) >= kChannelsPerBlock) {
            continue;
        }
        double v = entry.second;
        if (table.angles_in_radians) {
            v *= kRadToDeg;
        }
        out->elev_deg[static_cast<std::size_t>(entry.first)] = v;
    }
    return true;
}

}  // namespace hesai
}  // namespace lidar
}  // namespace autodriver
