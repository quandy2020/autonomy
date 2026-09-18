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

#include "autonomy/localization/atlas/runtime_config.hpp"

namespace autonomy::localization::atlas {
namespace estimate {

//! Which residual families to build in Local / Global Joint BA.
struct ResidualMask {
    bool vision = false;
    bool imu = false;
    bool lidar = false;
    bool odom = false;

    static ResidualMask FromRuntime(const RuntimeConfig& cfg) {
        ResidualMask m;
        m.vision = cfg.residuals.vision && cfg.flags.use_vision;
        m.imu = cfg.residuals.imu && cfg.flags.use_imu;
        m.lidar = cfg.residuals.lidar && cfg.flags.use_lidar;
        m.odom = cfg.residuals.odom && cfg.flags.use_odom;
        return m;
    }

    [[nodiscard]] bool any() const {
        return vision || imu || lidar || odom;
    }
};

//! Lidar residual material produced by sensor/lidar (Lightning ObsModel later).
struct LidarResidualBatch {
    int num_point_plane = 0;
    int num_point_point = 0;
    bool empty() const { return num_point_plane == 0 && num_point_point == 0; }
};

}  // namespace estimate
}  // namespace autonomy::localization::atlas
