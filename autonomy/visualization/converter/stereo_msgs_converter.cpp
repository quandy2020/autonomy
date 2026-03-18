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

#include "autonomy/visualization/converter/stereo_msgs_converter.hpp"

#include "autolink/autolink.hpp"
#include "autonomy/visualization/converter/converter_detail.hpp"

namespace autonomy {
namespace visualization {
namespace converter {
namespace impl {

using detail::ExtractFrameId;
using detail::ExtractTimestamp;
using detail::SetRawImageHeader;

// stereo_msgs conversion implementations
// Note: Currently stereo_msgs.proto is empty, so no implementations are
// provided. When DisparityImage and other stereo_msgs messages are added to the
// proto file, corresponding conversion implementations should be added here.
//
// Example implementation for DisparityImage (to be added when message type is
// available):
//
// foxglove::schemas::RawImage ToFoxgloveImpl(
//     const autonomy::commsgs::proto::stereo_msgs::DisparityImage& message) {
//     foxglove::schemas::RawImage raw_image;
//
//     // Extract header from the image field (as per ROS convention)
//     SetRawImageHeader(raw_image, ExtractTimestamp(message.image()),
//                       ExtractFrameId(message.image()));
//
//     // Copy image data from DisparityImage.image
//     if (message.has_image()) {
//         raw_image.width = message.image().width();
//         raw_image.height = message.image().height();
//         raw_image.encoding = message.image().encoding();
//         raw_image.step = message.image().step();
//
//         // Copy image data
//         if (message.image().data().size() > 0) {
//             raw_image.data.resize(message.image().data().size());
//             std::memcpy(raw_image.data.data(), message.image().data().data(),
//                        message.image().data().size());
//         }
//     }
//
//     AINFO << "Converted DisparityImage to RawImage: " << raw_image.width <<
//     "x"
//               << raw_image.height << ", encoding=" << raw_image.encoding;
//
//     return raw_image;
// }

}  // namespace impl
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy
