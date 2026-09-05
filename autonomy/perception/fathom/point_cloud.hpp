/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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
 * @brief Projection of metric depth into organized automsgs point clouds.
 */

#ifndef AUTONOMY_PERCEPTION_FATHOM_POINT_CLOUD_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_POINT_CLOUD_HPP_

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

/**
 * @brief Projects metric depth and a validity mask into organized XYZ points.
 *
 * Each valid `(u, v)` position becomes
 * `((u - cx) * z / fx, (v - cy) * z / fy, z)`. Masked, non-finite, and
 * non-positive depth pixels are represented by NaN in all three coordinates.
 *
 * @param depth_m `32FC1` metric depth image in meters.
 * @param mask `mono8` validity mask; non-zero values are valid.
 * @param camera_info Original-image intrinsics and matching dimensions.
 * @param cloud Output organized XYZ PointCloud2, cleared on entry and stamped
 *     from `depth_m` only on success.
 * @param error Optional diagnostic output, cleared on entry.
 * @return True when projection succeeds.
 */
bool ProjectDepth(const automsgs::msgs::sensor_msgs::Image& depth_m,
                  const automsgs::msgs::sensor_msgs::Image& mask,
                  const automsgs::msgs::sensor_msgs::CameraInfo& camera_info,
                  automsgs::msgs::sensor_msgs::PointCloud2* cloud,
                  std::string* error = nullptr);

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_POINT_CLOUD_HPP_
