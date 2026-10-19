/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file channels.hpp
 * @brief Bridge-side Autolink channel naming helpers.
 *
 * Complements ResolveChannel() in sensor_traits.hpp (modality defaults).
 * CameraInfoChannelForImage follows common ROS-style image → camera_info layout.
 */

#ifndef AUTODRIVER_BRIDGE_CHANNELS_HPP_
#define AUTODRIVER_BRIDGE_CHANNELS_HPP_

#include <string>
#include <string_view>

namespace autodriver {
namespace bridge {

/**
 * @brief Derive the camera_info channel from an image topic name.
 *
 * Examples:
 * - /camera/color/image_raw        → /camera/color/camera_info
 * - /camera/depth/image_rect_raw   → /camera/depth/camera_info
 * - other                          → @p image_channel + "/camera_info"
 *
 * @param[in] image_channel Image topic or channel name.
 * @return Matching camera_info channel name.
 */
inline std::string CameraInfoChannelForImage(const std::string& image_channel) {
  constexpr std::string_view kImageRaw = "/image_raw";
  constexpr std::string_view kImageRectRaw = "/image_rect_raw";
  if (image_channel.size() >= kImageRectRaw.size() &&
      image_channel.compare(image_channel.size() - kImageRectRaw.size(),
                            kImageRectRaw.size(), kImageRectRaw) == 0) {
    return image_channel.substr(0, image_channel.size() - kImageRectRaw.size()) +
           "/camera_info";
  }
  if (image_channel.size() >= kImageRaw.size() &&
      image_channel.compare(image_channel.size() - kImageRaw.size(),
                            kImageRaw.size(), kImageRaw) == 0) {
    return image_channel.substr(0, image_channel.size() - kImageRaw.size()) +
           "/camera_info";
  }
  return image_channel + "/camera_info";
}

/**
 * @brief Derive a lidar IMU channel from a PointCloud2 topic name.
 *
 * Examples:
 * - /lidar/jt128/points → /lidar/jt128/imu
 * - /lidar/hesai        → /lidar/hesai/imu
 *
 * @param[in] points_channel Point cloud topic or channel name.
 * @return Matching IMU channel name, or empty when @p points_channel is empty.
 */
inline std::string LidarImuChannelForPoints(const std::string& points_channel) {
  if (points_channel.empty()) {
    return {};
  }
  constexpr std::string_view kPoints = "/points";
  if (points_channel.size() >= kPoints.size() &&
      points_channel.compare(points_channel.size() - kPoints.size(),
                             kPoints.size(), kPoints) == 0) {
    return points_channel.substr(0, points_channel.size() - kPoints.size()) +
           "/imu";
  }
  return points_channel + "/imu";
}

}  // namespace bridge
}  // namespace autodriver

#endif  // AUTODRIVER_BRIDGE_CHANNELS_HPP_
