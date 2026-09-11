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
 * @file beam_calibration_yaml.cpp
 * @brief Shared YAML loader for lasers[].vert_correction (Velodyne / Hesai)
 *        (implementation).
 */

#include "autodriver/lidar/beam_calibration_yaml.hpp"

#include <yaml-cpp/yaml.h>

namespace autodriver {
namespace lidar {
namespace {

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

bool LoadLaserVertCorrectionsYaml(const std::string& path,
                                  LaserVertCorrectionTable* out,
                                  std::string* error) {
    if (out == nullptr) {
        if (error != nullptr) {
            *error = "null LaserVertCorrectionTable";
        }
        return false;
    }
    out->by_id.clear();
    out->angles_in_radians = false;
    try {
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node lasers = root["lasers"] ? root["lasers"] : root;
        if (!lasers || !lasers.IsSequence()) {
            if (error != nullptr) {
                *error = "missing lasers sequence";
            }
            return false;
        }
        out->angles_in_radians = AnglesAreRadians(root);
        for (const auto& node : lasers) {
            const int id = node["laser_id"] ? node["laser_id"].as<int>() : -1;
            if (id < 0 || !node["vert_correction"]) {
                continue;
            }
            out->by_id[id] = node["vert_correction"].as<double>();
        }
        if (out->by_id.empty()) {
            if (error != nullptr) {
                *error = "no laser vert_correction entries";
            }
            return false;
        }
        return true;
    } catch (const YAML::Exception& ex) {
        if (error != nullptr) {
            *error = ex.what();
        }
        return false;
    }
}

}  // namespace lidar
}  // namespace autodriver
