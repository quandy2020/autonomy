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

#include "autodriver/lidar/velodyne/calibration.hpp"

#include <cmath>
#include <map>
#include <algorithm>

#include <yaml-cpp/yaml.h>

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
        cal.vert_correction_rad[static_cast<std::size_t>(i)] = deg[i] * kDegToRad;
    }
    return cal;
}

bool LoadBeamCalibrationYaml(const std::string& path, BeamCalibration* out,
                             std::string* error) {
    if (out == nullptr) {
        if (error) {
            *error = "null BeamCalibration";
        }
        return false;
    }
    try {
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node lasers = root["lasers"] ? root["lasers"] : root;
        if (!lasers || !lasers.IsSequence()) {
            if (error) {
                *error = "missing lasers sequence";
            }
            return false;
        }
        std::map<int, double> by_id;
        int max_id = -1;
        for (const auto& node : lasers) {
            const int id = node["laser_id"] ? node["laser_id"].as<int>() : -1;
            if (id < 0 || !node["vert_correction"]) {
                continue;
            }
            by_id[id] = node["vert_correction"].as<double>();
            max_id = std::max(max_id, id);
        }
        if (by_id.empty()) {
            if (error) {
                *error = "no laser vert_correction entries";
            }
            return false;
        }
        out->vert_correction_rad.assign(
            static_cast<std::size_t>(max_id + 1), 0.0);
        for (const auto& entry : by_id) {
            out->vert_correction_rad[static_cast<std::size_t>(entry.first)] =
                entry.second;
        }
        return true;
    } catch (const YAML::Exception& ex) {
        if (error) {
            *error = ex.what();
        }
        return false;
    }
}

}  // namespace velodyne
}  // namespace lidar
}  // namespace autodriver
