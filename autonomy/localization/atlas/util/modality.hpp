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

#pragma once

#include <string>

namespace autonomy::localization::atlas {
namespace common {

//! Single AtlasSystem modality = sensor / residual mask (not a second SLAM).
enum class Modality {
    kVo = 0,
    kVio,
    kLo,
    kLio,
    kLivo,
    kWio,     //!< odom + imu
    kLwio,    //!< lidar + odom + imu
    kLvwio,   //!< lidar + vision + odom + imu
};

struct ModalityFlags {
    bool use_vision = false;
    bool use_lidar = false;
    bool use_imu = false;
    bool use_odom = false;   //!< wheel / external odometry (sensor/odom)
    bool use_joint = false;  //!< Local/Global joint residuals (multi-sensor)
};

Modality ParseModality(const std::string& name);
std::string ModalityName(Modality m);
ModalityFlags FlagsFor(Modality m);

bool IsAtlasModalityName(const std::string& name);

}  // namespace common
}  // namespace autonomy::localization::atlas
