/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_IMU_CONFIG_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_IMU_CONFIG_HPP_

#include "autonomy/localization/atlas/type.hpp"

#include <string>

#include "yaml-cpp/yaml.h"

namespace autonomy::localization::atlas {
namespace imu {

//! IMU noise / extrinsic parameters loaded from YAML `IMU:` section.
struct config {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool enabled = false;

    //! IMU rate [Hz] (informational; integration uses measurement timestamps)
    double frequency = 200.0;

    //! Noise density: accelerometer [m/s^2/sqrt(Hz)], gyro [rad/s/sqrt(Hz)]
    double noise_acc = 0.01;
    double noise_gyro = 0.001;

    //! Random-walk: accelerometer [m/s^3/sqrt(Hz)], gyro [rad/s^2/sqrt(Hz)]
    double random_walk_acc = 0.001;
    double random_walk_gyro = 0.0001;

    //! Gravity magnitude [m/s^2]
    double gravity_magnitude = 9.81;

    //! Transform IMU -> camera (T_c_b): p_c = R_c_b * p_b + t_c_b
    Mat33_t R_c_b = Mat33_t::Identity();
    Vec3_t t_c_b = Vec3_t::Zero();

    //! IMU timestamp = image_timestamp + time_offset (seconds)
    double time_offset = 0.0;

    //! Max |imu_t - image_t| when syncing for diagnostics
    double time_sync_slop = 0.02;

    //! Max IMU buffer size
    std::size_t buffer_capacity = 10000;

    //! Minimum keyframes before attempting inertial initialization
    unsigned int init_min_keyframes = 10;

    static config from_yaml(const YAML::Node& node);
    Mat44_t T_c_b() const;
};

}  // namespace imu
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_IMU_CONFIG_HPP_
