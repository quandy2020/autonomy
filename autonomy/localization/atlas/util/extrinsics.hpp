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

namespace autonomy::localization::atlas {
namespace common {

//! Sensor extrinsics / time sync for multimodal fusion (YAML-loadable later).
struct Extrinsics {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Lidar → camera: p_c = R * p_l + t
    Mat33_t R_c_l = Mat33_t::Identity();
    Vec3_t t_c_l = Vec3_t::Zero();

    //! Base → wheel odometry frame
    Mat33_t R_b_w = Mat33_t::Identity();
    Vec3_t t_b_w = Vec3_t::Zero();

    //! IMU / lidar / camera time offsets (seconds): sensor_time = host_time + offset
    double time_offset_imu = 0.0;
    double time_offset_lidar = 0.0;
    double time_offset_cam = 0.0;
    double time_offset_wheel = 0.0;

    Mat44_t T_c_l() const {
        Mat44_t T = Mat44_t::Identity();
        T.block<3, 3>(0, 0) = R_c_l;
        T.block<3, 1>(0, 3) = t_c_l;
        return T;
    }

    Mat44_t T_b_w() const {
        Mat44_t T = Mat44_t::Identity();
        T.block<3, 3>(0, 0) = R_b_w;
        T.block<3, 1>(0, 3) = t_b_w;
        return T;
    }
};

}  // namespace common
}  // namespace autonomy::localization::atlas
