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

#include "autonomy/localization/atlas/type.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace autonomy::localization::atlas {
namespace sensor {

struct ImuSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp = 0.0;
    Vec3_t acc = Vec3_t::Zero();
    Vec3_t gyro = Vec3_t::Zero();
};

struct ImageSample {
    double timestamp = 0.0;
    // Raw buffers owned by bridge / OpenCV Mat lifetime elsewhere.
    std::uint64_t frame_id = 0;
};

struct CloudSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp = 0.0;
    std::uint64_t frame_id = 0;
    //! Body-frame points (filled by lidar driver / bridge).
    std::vector<Vec3_t> points_body;
};

struct OdomSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp = 0.0;
    double v_mps = 0.0;
    double yaw_rate_rps = 0.0;
    Mat44_t T_delta = Mat44_t::Identity();
};

}  // namespace sensor
}  // namespace autonomy::localization::atlas
