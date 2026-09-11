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
 * @brief Velodyne beam calibration (vert_correction radians) (implementation).
 */

#include "autodriver/lidar/velodyne/calibration.hpp"

#include <algorithm>
#include <cmath>

#include "autodriver/lidar/beam_calibration_yaml.hpp"

namespace autodriver {
namespace lidar {
namespace velodyne {
namespace {

constexpr double kDegToRad = 0.017453292519943295;

}  // namespace

BeamCalibration DefaultVlp16Calibration() {
    BeamCalibration cal;
    const double deg[16] = {-15.0, 1.0,  -13.0, 3.0,  -11.0, 5.0,  -9.0, 7.0,
                            -7.0,  9.0,  -5.0,  11.0, -3.0,  13.0, -1.0, 15.0};
    cal.vert_correction_rad.resize(16);
    for (int i = 0; i < 16; ++i) {
        cal.vert_correction_rad[static_cast<std::size_t>(i)] =
            deg[i] * kDegToRad;
    }
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
    int max_id = -1;
    for (const auto& entry : table.by_id) {
        max_id = std::max(max_id, entry.first);
    }
    out->vert_correction_rad.assign(static_cast<std::size_t>(max_id + 1), 0.0);
    for (const auto& entry : table.by_id) {
        // Velodyne YAML is documented as radians; ignore unit flag.
        out->vert_correction_rad[static_cast<std::size_t>(entry.first)] =
            entry.second;
    }
    return true;
}

}  // namespace velodyne
}  // namespace lidar
}  // namespace autodriver
