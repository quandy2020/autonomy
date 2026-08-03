/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/common/time.hpp"
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud.pb.h>

namespace autonomy {
namespace sensor {

using ImuData = automsgs::msgs::sensor_msgs::Imu;
using ImuProto = automsgs::msgs::sensor_msgs::Imu;

// Converts 'data' to a proto::sensor_msgs::IMU.
ImuProto ToProto(const ImuData& data);

// Converts 'proto' to IMU.
ImuData FromProto(const ImuProto& proto);

}  // namespace sensor
}  // namespace autonomy