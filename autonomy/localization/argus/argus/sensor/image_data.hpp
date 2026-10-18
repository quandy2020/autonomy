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
 * @file image_data.hpp
 * @brief Camera frame for FAST-LIVO2 (OpenCV Mat; I/O via automsgs).
 *
 * After decode, `image` is contiguous:
 *   - mono8  → CV_8UC1
 *   - color  → CV_8UC3 BGR (rgb8 / rgba8 / bgra8 are converted)
 */

#ifndef ARGUS_SENSOR_IMAGE_DATA_HPP_
#define ARGUS_SENSOR_IMAGE_DATA_HPP_

#include <string>

#include <opencv2/core.hpp>

#include <automsgs/msgs/sensor_msgs/compressed_image.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>

#include "argus/common/macros.hpp"

namespace argus {
namespace sensor {

struct ImageData {
    ARGUS_SHARED_PTR_DEFINITIONS(ImageData);

    double timestamp = 0.0;
    std::string frame_id;
    /** @brief Contiguous Mat: mono8 (CV_8UC1) or BGR (CV_8UC3). */
    cv::Mat image;
    /** @brief Encoding of `image`: "mono8" or "bgr8". */
    std::string encoding = "bgr8";

    bool IsEmpty() const {
        return image.empty();
    }
    bool IsColor() const {
        return image.channels() == 3;
    }

    /** @brief Deep copy (Mat clone). */
    ImageData Clone() const;

    /** @brief Gray view: clone if already mono, else BGR→GRAY clone. */
    cv::Mat ToGray() const;
};

/** @brief Decode sensor_msgs/Image → ImageData. Empty Mat on failure. */
ImageData FromProto(const ::automsgs::msgs::sensor_msgs::Image& msg);

/** @brief Decode CompressedImage (jpeg/png/…) via OpenCV imdecode. */
ImageData FromProto(const ::automsgs::msgs::sensor_msgs::CompressedImage& msg);

/** @brief Encode ImageData → sensor_msgs/Image (mono8 or bgr8). */
::automsgs::msgs::sensor_msgs::Image ToProto(const ImageData& data);

/**
 * @brief Encode a Mat with stamp / frame.
 *
 * CV_8UC1 → mono8; CV_8UC3 → bgr8; other types returned empty.
 */
::automsgs::msgs::sensor_msgs::Image ToProto(const cv::Mat& image,
                                             double timestamp_sec,
                                             const std::string& frame_id = "");

/**
 * @brief Encode ImageData → CompressedImage (default jpeg).
 * @param format OpenCV imencode extension without leading dot, e.g.
 * "jpeg","png".
 * @param jpeg_quality 0–100 (ignored for non-jpeg).
 */
::automsgs::msgs::sensor_msgs::CompressedImage ToCompressedProto(
    const ImageData& data, const std::string& format = "jpeg",
    int jpeg_quality = 95);

::automsgs::msgs::sensor_msgs::CompressedImage ToCompressedProto(
    const cv::Mat& image, double timestamp_sec,
    const std::string& frame_id = "", const std::string& format = "jpeg",
    int jpeg_quality = 95);

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_IMAGE_DATA_HPP_
