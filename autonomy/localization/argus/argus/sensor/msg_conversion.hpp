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
 * @file msg_conversion.hpp
 * @brief automsgs sensor messages ↔ Argus sensor values.
 *
 * Point times follow Cartographer: after conversion the header time is the
 * last return, and each point's time is seconds relative to that return
 * (<= 0).
 */

#ifndef ARGUS_SENSOR_MSG_CONVERSION_HPP_
#define ARGUS_SENSOR_MSG_CONVERSION_HPP_

#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "sensor/imu_data.hpp"
#include "sensor/point_cloud.hpp"
#include "sensor/timed_point_cloud_data.hpp"

namespace argus {
namespace sensor {

TimedPointCloudData FromProto(
    const ::automsgs::msgs::sensor_msgs::PointCloud2& msg);
::automsgs::msgs::sensor_msgs::PointCloud2 ToProto(
    const TimedPointCloudData& cloud, const std::string& frame_id);

TimedPointCloudData FromProto(
    const ::automsgs::msgs::sensor_msgs::LaserScan& msg);

PointCloud PointCloudFromProto(
    const ::automsgs::msgs::sensor_msgs::PointCloud2& msg);

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_MSG_CONVERSION_HPP_
