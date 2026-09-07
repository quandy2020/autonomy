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

#include "autodriver/lidar/hesai/calibration.hpp"

#include <cmath>
#include <map>
#include <string>

#include <yaml-cpp/yaml.h>

namespace autodriver {
namespace lidar {
namespace hesai {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

bool AnglesAreRadians(const YAML::Node& root) {
    if (root["unit"]) {
        const std::string unit = root["unit"].as<std::string>();
        return unit == "rad" || unit == "radian" || unit == "radians";
    }
    if (root["angles_in_radians"]) {
        return root["angles_in_radians"].as<bool>();
    }
    return false;
}

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
        const bool as_rad = AnglesAreRadians(root);
        std::map<int, double> by_id;
        for (const auto& node : lasers) {
            const int id = node["laser_id"] ? node["laser_id"].as<int>() : -1;
            if (id < 0 ||
                static_cast<std::size_t>(id) >= kChannelsPerBlock ||
                !node["vert_correction"]) {
                continue;
            }
            by_id[id] = node["vert_correction"].as<double>();
        }
        if (by_id.size() < kChannelsPerBlock) {
            if (error) {
                *error = "need 32 laser vert_correction entries for XT32";
            }
            return false;
        }
        *out = DefaultXt32Calibration();
        for (const auto& entry : by_id) {
            double v = entry.second;
            if (as_rad) {
                v *= kRadToDeg;
            }
            out->elev_deg[static_cast<std::size_t>(entry.first)] = v;
        }
        return true;
    } catch (const YAML::Exception& ex) {
        if (error) {
            *error = ex.what();
        }
        return false;
    }
}

}  // namespace hesai
}  // namespace lidar
}  // namespace autodriver
