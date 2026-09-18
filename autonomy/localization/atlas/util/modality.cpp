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

#include "autonomy/localization/atlas/util/modality.hpp"

#include <algorithm>
#include <cctype>

namespace autonomy::localization::atlas {
namespace common {
namespace {

std::string ToLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

}  // namespace

Modality ParseModality(const std::string& name) {
    const std::string n = ToLower(name);
    if (n == "vo") {
        return Modality::kVo;
    }
    if (n == "vio" || n == "atlas" || n == "openvslam") {
        return Modality::kVio;
    }
    if (n == "lo") {
        return Modality::kLo;
    }
    if (n == "lio") {
        return Modality::kLio;
    }
    if (n == "livo") {
        return Modality::kLivo;
    }
    if (n == "wio") {
        return Modality::kWio;
    }
    if (n == "lwio") {
        return Modality::kLwio;
    }
    if (n == "lvwio" || n == "liwvo") {
        return Modality::kLvwio;
    }
    return Modality::kVio;
}

std::string ModalityName(Modality m) {
    switch (m) {
        case Modality::kVo:
            return "vo";
        case Modality::kVio:
            return "vio";
        case Modality::kLo:
            return "lo";
        case Modality::kLio:
            return "lio";
        case Modality::kLivo:
            return "livo";
        case Modality::kWio:
            return "wio";
        case Modality::kLwio:
            return "lwio";
        case Modality::kLvwio:
            return "lvwio";
    }
    return "vio";
}

ModalityFlags FlagsFor(Modality m) {
    ModalityFlags f;
    switch (m) {
        case Modality::kVo:
            f.use_vision = true;
            break;
        case Modality::kVio:
            f.use_vision = true;
            f.use_imu = true;
            break;
        case Modality::kLo:
            f.use_lidar = true;
            f.use_joint = true;
            break;
        case Modality::kLio:
            f.use_lidar = true;
            f.use_imu = true;
            f.use_joint = true;
            break;
        case Modality::kLivo:
            f.use_vision = true;
            f.use_lidar = true;
            f.use_imu = true;
            f.use_joint = true;
            break;
        case Modality::kWio:
            f.use_odom = true;
            f.use_imu = true;
            f.use_joint = true;
            break;
        case Modality::kLwio:
            f.use_lidar = true;
            f.use_odom = true;
            f.use_imu = true;
            f.use_joint = true;
            break;
        case Modality::kLvwio:
            f.use_vision = true;
            f.use_lidar = true;
            f.use_odom = true;
            f.use_imu = true;
            f.use_joint = true;
            break;
    }
    return f;
}

bool IsAtlasModalityName(const std::string& name) {
    const std::string n = ToLower(name);
    return n == "vo" || n == "vio" || n == "lo" || n == "lio" || n == "livo"
           || n == "wio" || n == "lwio" || n == "lvwio" || n == "liwvo"
           || n == "atlas" || n == "openvslam";
}

}  // namespace common
}  // namespace autonomy::localization::atlas
