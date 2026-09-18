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

//! frontend/lio/measure_group — synced lidar + IMU window for LIO deskew.

#include "autonomy/localization/atlas/sensor/types.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <deque>
#include <vector>

namespace autonomy::localization::atlas {
namespace frontend {
namespace lio {

struct MeasureGroup {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double lidar_begin_time = 0.0;
    double lidar_end_time = 0.0;
    std::vector<Vec3_t> points_body;
    //! Optional per-point relative time in [0,1] (scan begin→end). Empty → no deskew.
    std::vector<double> point_time_rel;
    std::deque<sensor::ImuSample> imu;
};

}  // namespace lio
}  // namespace frontend
}  // namespace autonomy::localization::atlas
