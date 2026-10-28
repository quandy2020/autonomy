// Copyright 2026 The Openbot Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_SENSOR_IMAGE_DATA_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_SENSOR_IMAGE_DATA_HPP_

#include <automsgs/msgs/sensor_msgs/image.pb.h>

#include <cstdint>
#include <string>
#include <vector>

namespace autonomy::localization::atlas {

/** @brief Validated raw image; row padding is retained in data. */
struct ImageData {
    std::int64_t timestamp_ns = 0;
    std::string frame_id;
    std::uint32_t height = 0;
    std::uint32_t width = 0;
    std::string encoding;
    bool is_bigendian = false;
    std::uint32_t step = 0;
    std::vector<std::uint8_t> data;
};

/** @brief Validates encoding, dimensions, stride, byte order and payload size.
 */
bool ValidateImageData(const ImageData& image, std::string* error = nullptr);

/** @brief Converts AutoMsgs to ImageData without modifying output on failure.
 */
bool FromProto(const ::automsgs::msgs::sensor_msgs::Image& message,
               ImageData* image, std::string* error = nullptr);

/** @brief Converts validated ImageData without modifying output on failure. */
bool ToProto(const ImageData& image,
             ::automsgs::msgs::sensor_msgs::Image* message,
             std::string* error = nullptr);

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_SENSOR_IMAGE_DATA_HPP_
