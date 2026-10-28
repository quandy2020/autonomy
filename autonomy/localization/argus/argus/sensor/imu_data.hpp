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

/**
 * @file imu_data.hpp
 * @brief automsgs Imu conversion for @ref imu_types.hpp.
 */

#ifndef ARGUS_SENSOR_IMU_DATA_HPP_
#define ARGUS_SENSOR_IMU_DATA_HPP_

#include <automsgs/msgs/sensor_msgs/imu.pb.h>

#include "sensor/imu_types.hpp"

namespace argus {
namespace sensor {

/** @brief automsgs `sensor_msgs/Imu` → @ref ImuData. */
ImuData FromProto(const ::automsgs::msgs::sensor_msgs::Imu& msg);

/** @brief @ref ImuData → automsgs `sensor_msgs/Imu`. */
::automsgs::msgs::sensor_msgs::Imu ToProto(const ImuData& data);

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_IMU_DATA_HPP_
