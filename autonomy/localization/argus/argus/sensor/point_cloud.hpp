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
 * @file point_cloud.hpp
 * @brief PointCloud2 ↔ Argus cloud conversion (FAST-LIVO2 I/O).
 */

#ifndef ARGUS_SENSOR_POINT_CLOUD_HPP_
#define ARGUS_SENSOR_POINT_CLOUD_HPP_

#include <string>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "argus/sensor/point_types.hpp"

namespace argus {
namespace sensor {

/**
 * @brief Decode PointCloud2 into an Argus XYZINormal cloud.
 * @param msg Source protobuf message.
 * @param cloud Output cloud (must be non-null).
 */
void FromProto(const ::automsgs::msgs::sensor_msgs::PointCloud2& msg,
               PointCloud<PointXYZINormal>* cloud);
/**
 * @brief Decode PointCloud2 into an Argus XYZI cloud.
 * @param msg Source protobuf message.
 * @param cloud Output cloud (must be non-null).
 */
void FromProto(const ::automsgs::msgs::sensor_msgs::PointCloud2& msg,
               PointCloud<PointXYZI>* cloud);
/**
 * @brief Decode PointCloud2 into an Argus XYZRGB cloud.
 * @param msg Source protobuf message.
 * @param cloud Output cloud (must be non-null).
 */
void FromProto(const ::automsgs::msgs::sensor_msgs::PointCloud2& msg,
               PointCloud<PointXYZRGB>* cloud);

/**
 * @brief Encode Argus XYZINormal cloud to PointCloud2.
 * @param cloud Input cloud.
 * @param stamp_sec Message timestamp (seconds).
 * @param frame_id TF frame id string.
 * @return Populated PointCloud2 message.
 */
::automsgs::msgs::sensor_msgs::PointCloud2 ToProto(
    const PointCloud<PointXYZINormal>& cloud, double stamp_sec,
    const std::string& frame_id);
/**
 * @brief Encode Argus XYZRGB cloud to PointCloud2.
 * @param cloud Input cloud.
 * @param stamp_sec Message timestamp (seconds).
 * @param frame_id TF frame id string.
 * @return Populated PointCloud2 message.
 */
::automsgs::msgs::sensor_msgs::PointCloud2 ToProto(
    const PointCloud<PointXYZRGB>& cloud, double stamp_sec,
    const std::string& frame_id);
/**
 * @brief Encode Argus XYZI cloud to PointCloud2.
 * @param cloud Input cloud.
 * @param stamp_sec Message timestamp (seconds).
 * @param frame_id TF frame id string.
 * @return Populated PointCloud2 message.
 */
::automsgs::msgs::sensor_msgs::PointCloud2 ToProto(
    const PointCloud<PointXYZI>& cloud, double stamp_sec,
    const std::string& frame_id);

/**
 * @brief Fill an existing PointCloud2 from XYZINormal cloud.
 * @param cloud Input cloud.
 * @param msg Output message (must be non-null).
 * @param stamp_sec Message timestamp (seconds).
 * @param frame_id TF frame id string.
 */
void ToProto(const PointCloud<PointXYZINormal>& cloud,
             ::automsgs::msgs::sensor_msgs::PointCloud2* msg, double stamp_sec,
             const std::string& frame_id);
/** @copydoc ToProto(const PointCloud<PointXYZINormal>&, PointCloud2*, double,
 * const std::string&) */
void ToProto(const PointCloud<PointXYZRGB>& cloud,
             ::automsgs::msgs::sensor_msgs::PointCloud2* msg, double stamp_sec,
             const std::string& frame_id);
/** @copydoc ToProto(const PointCloud<PointXYZINormal>&, PointCloud2*, double,
 * const std::string&) */
void ToProto(const PointCloud<PointXYZI>& cloud,
             ::automsgs::msgs::sensor_msgs::PointCloud2* msg, double stamp_sec,
             const std::string& frame_id);

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_POINT_CLOUD_HPP_
