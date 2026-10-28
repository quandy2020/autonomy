// Copyright 2026 The Openbot Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_SENSOR_IMU_DATA_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_SENSOR_IMU_DATA_HPP_

#include <automsgs/msgs/sensor_msgs/imu.pb.h>

#include <array>
#include <cstdint>
#include <string>

namespace autonomy::localization::atlas {

/**
 * @brief IMU sample expressed in frame_id using SI units.
 * @note timestamp_ns uses the AutoMsgs header clock epoch.
 */
struct ImuData {
    std::int64_t timestamp_ns = 0;
    std::string frame_id;
    bool has_orientation = false;
    std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
    bool has_orientation_covariance = false;
    std::array<double, 9> orientation_covariance{};
    std::array<double, 3> linear_acceleration_mps2{};
    bool has_linear_acceleration_covariance = false;
    std::array<double, 9> linear_acceleration_covariance{};
    std::array<double, 3> angular_velocity_radps{};
    bool has_angular_velocity_covariance = false;
    std::array<double, 9> angular_velocity_covariance{};
};

/** @brief Converts AutoMsgs without modifying @p data on failure. */
bool FromProto(const ::automsgs::msgs::sensor_msgs::Imu& message, ImuData* data,
               std::string* error = nullptr);

/** @brief Converts ImuData without modifying @p message on failure. */
bool ToProto(const ImuData& data, ::automsgs::msgs::sensor_msgs::Imu* message,
             std::string* error = nullptr);

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_SENSOR_IMU_DATA_HPP_
