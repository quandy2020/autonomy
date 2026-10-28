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
 * @file point_cloud_data.hpp
 * @brief automsgs PointCloud2 conversion for @ref point_types.hpp clouds.
 */

#ifndef ARGUS_SENSOR_POINT_CLOUD_DATA_HPP_
#define ARGUS_SENSOR_POINT_CLOUD_DATA_HPP_

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include <string>

#include "sensor/point_types.hpp"

namespace argus {
namespace sensor {

/** @brief Decode PointCloud2 into an XYZINormal cloud. */
void FromProto(const ::automsgs::msgs::sensor_msgs::PointCloud2& msg,
               LidarCloud<PointXYZINormal>* cloud);

/** @brief Encode an Argus XYZI cloud as a packed AutoMsgs PointCloud2. */
void ToProto(const LidarCloud<PointXYZINormal>& cloud,
             ::automsgs::msgs::sensor_msgs::PointCloud2* msg, double stamp_sec,
             const std::string& frame_id);

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_POINT_CLOUD_DATA_HPP_
