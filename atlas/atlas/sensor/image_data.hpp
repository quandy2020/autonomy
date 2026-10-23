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
 * @brief Image samples and automsgs Image / CompressedImage I/O.
 *
 * Feature extraction stays in the visual frontend. This type only carries
 * pixels and, for compressed messages, decodes them into raw bytes.
 */

#ifndef ATLAS_SENSOR_IMAGE_DATA_HPP_
#define ATLAS_SENSOR_IMAGE_DATA_HPP_

#include <automsgs/msgs/sensor_msgs/compressed_image.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>

#include <cstdint>
#include <string>
#include <vector>

namespace atlas {
namespace sensor {

struct ImageData {
  double time = 0.0;
  std::string frame_id;
  std::uint32_t height = 0;
  std::uint32_t width = 0;
  std::string encoding;
  std::uint32_t step = 0;
  std::vector<std::uint8_t> data;
};

struct CompressedImageData {
  double time = 0.0;
  std::string frame_id;
  std::string format;
  std::vector<std::uint8_t> data;
};

ImageData FromProto(const ::automsgs::msgs::sensor_msgs::Image& msg);
::automsgs::msgs::sensor_msgs::Image ToProto(const ImageData& image);

CompressedImageData FromProto(
    const ::automsgs::msgs::sensor_msgs::CompressedImage& msg);
::automsgs::msgs::sensor_msgs::CompressedImage ToProto(
    const CompressedImageData& image);

/**
 * @brief Decode a compressed buffer into raw pixels.
 * Color images are `bgr8`. Single-channel images are `mono8`.
 * @return false when the buffer is not a decodable image. `out` is unchanged.
 */
bool DecodeCompressedImage(const CompressedImageData& compressed,
                           ImageData* out);

}  // namespace sensor
}  // namespace atlas

#endif  // ATLAS_SENSOR_IMAGE_DATA_HPP_
