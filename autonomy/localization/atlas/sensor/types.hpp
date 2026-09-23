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
 * @file types.hpp
 * @brief Atlas frontend sensor packet built from automsgs `sensor_msgs`.
 *
 * Raw observations use standard message types from
 * `automsgs/proto/msgs/sensor_msgs` (no Atlas-specific Image/IMU/LiDAR
 * layouts). `SensorData` only aggregates those messages for one tracking step.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SENSOR_TYPES_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SENSOR_TYPES_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>

namespace autonomy {
namespace localization {
namespace atlas {

/** @brief Standard IMU message (`sensor_msgs/Imu`). */
using Imu = ::automsgs::msgs::sensor_msgs::Imu;

/** @brief Standard uncompressed image (`sensor_msgs/Image`). */
using Image = ::automsgs::msgs::sensor_msgs::Image;

/** @brief Standard point cloud (`sensor_msgs/PointCloud2`). */
using PointCloud2 = ::automsgs::msgs::sensor_msgs::PointCloud2;

/**
 * @brief Header stamp as seconds (floating point).
 * @param header Message header; missing stamp yields 0.
 * @return `sec + nanosec * 1e-9`.
 */
inline double HeaderStampSec(const ::automsgs::msgs::std_msgs::Header& header) {
    if (!header.has_stamp()) {
        return 0.0;
    }
    return static_cast<double>(header.stamp().sec()) +
           1e-9 * static_cast<double>(header.stamp().nanosec());
}

/**
 * @brief Map a common `sensor_msgs/Image` encoding to an OpenCV type.
 * @param encoding Encoding string (`mono8`, `bgr8`, `32FC1`, …).
 * @return OpenCV type, or -1 if unsupported.
 */
inline int ImageEncodingToCvType(const std::string& encoding) {
    if (encoding == "mono8" || encoding == "8UC1") {
        return CV_8UC1;
    }
    if (encoding == "mono16" || encoding == "16UC1") {
        return CV_16UC1;
    }
    if (encoding == "bgr8" || encoding == "rgb8" || encoding == "8UC3") {
        return CV_8UC3;
    }
    if (encoding == "bgra8" || encoding == "rgba8" || encoding == "8UC4") {
        return CV_8UC4;
    }
    if (encoding == "32FC1") {
        return CV_32FC1;
    }
    if (encoding == "16SC1") {
        return CV_16SC1;
    }
    return -1;
}

/**
 * @brief Wrap `sensor_msgs/Image` pixels as a contiguous `cv::Mat` clone.
 * @param image Input image message.
 * @return Cloned matrix; empty if dimensions/encoding/data are invalid.
 *
 * @note Prefer encodings `mono8` / `bgr8` / `rgb8` for tracking images and
 *       `32FC1` for depth. `rgb8` is left as CV_8UC3 (caller may convert BGR).
 */
inline cv::Mat ImageToCvMat(const Image& image) {
    if (image.height() == 0 || image.width() == 0 || image.data().empty()) {
        return cv::Mat();
    }
    const int type = ImageEncodingToCvType(image.encoding());
    if (type < 0) {
        return cv::Mat();
    }
    auto* ptr = reinterpret_cast<void*>(
        const_cast<char*>(image.data().data()));
    const int rows = static_cast<int>(image.height());
    const int cols = static_cast<int>(image.width());
    // protobuf `bytes` is contiguous; clone so the Mat outlives the message.
    if (image.step() == 0) {
        return cv::Mat(rows, cols, type, ptr).clone();
    }
    return cv::Mat(rows, cols, type, ptr,
                   static_cast<size_t>(image.step()))
        .clone();
}

/**
 * @struct autonomy::localization::atlas::SensorData
 * @brief One tracking-step multi-sensor packet (automsgs `sensor_msgs` only).
 *
 * Fields are standard messages; `has_*` flags mark which slots are populated
 * for this step (proto3 messages have no native optional presence for these).
 *
 * @code{.cpp}
 * SensorData packet;
 * packet.image = std::move(left_img);   // sensor_msgs/Image
 * packet.has_image = true;
 * packet.imu.push_back(imu_msg);        // sensor_msgs/Imu
 * packet.has_imu = true;
 * frontend->Process(packet);
 * @endcode
 */
struct SensorData {
    std::vector<Imu> imu;       ///< IMU samples since the previous image.
    Image image;                ///< Left / monocular image.
    Image image_right;          ///< Right image (stereo).
    Image depth;                ///< Depth image (RGB-D), typically `32FC1`.
    PointCloud2 lidar;          ///< LiDAR cloud (`sensor_msgs/PointCloud2`).
    bool has_image = false;     ///< Whether `image` is valid for this step.
    bool has_image_right = false;  ///< Whether `image_right` is valid.
    bool has_depth = false;     ///< Whether `depth` is valid.
    bool has_lidar = false;     ///< Whether `lidar` is valid.
    bool has_imu = false;       ///< Whether `imu` is non-empty and valid.
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SENSOR_TYPES_HPP_
