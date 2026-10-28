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

#include "atlas/sensor/image_data.hpp"

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <utility>

namespace autonomy::localization::atlas {
namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

bool Fail(std::string* error, std::string message) {
    if (error != nullptr) {
        *error = std::move(message);
    }
    return false;
}

bool CheckedMultiply(std::size_t lhs, std::size_t rhs, std::size_t* result) {
    if (lhs != 0 && rhs > std::numeric_limits<std::size_t>::max() / lhs) {
        return false;
    }
    *result = lhs * rhs;
    return true;
}

bool TimestampFromProto(const ::automsgs::msgs::builtin_interfaces::Time& stamp,
                        std::int64_t* timestamp_ns, std::string* error) {
    if (stamp.nanosec() >= kNanosecondsPerSecond) {
        return Fail(error, "header stamp nanosec must be less than 1e9");
    }
    *timestamp_ns =
        static_cast<std::int64_t>(stamp.sec()) * kNanosecondsPerSecond +
        static_cast<std::int64_t>(stamp.nanosec());
    return true;
}

bool TimestampToProto(std::int64_t timestamp_ns,
                      ::automsgs::msgs::builtin_interfaces::Time* stamp,
                      std::string* error) {
    std::int64_t sec = timestamp_ns / kNanosecondsPerSecond;
    std::int64_t nanosec = timestamp_ns % kNanosecondsPerSecond;
    if (nanosec < 0) {
        --sec;
        nanosec += kNanosecondsPerSecond;
    }
    if (sec < std::numeric_limits<std::int32_t>::min() ||
        sec > std::numeric_limits<std::int32_t>::max()) {
        return Fail(error, "timestamp seconds do not fit AutoMsgs int32 sec");
    }
    stamp->set_sec(static_cast<std::int32_t>(sec));
    stamp->set_nanosec(static_cast<std::uint32_t>(nanosec));
    return true;
}

bool ParseGenericEncoding(std::string_view encoding,
                          std::size_t* bytes_per_pixel) {
    std::size_t cursor = 0;
    std::uint32_t bit_depth = 0;
    while (cursor < encoding.size() &&
           std::isdigit(static_cast<unsigned char>(encoding[cursor]))) {
        const auto digit = static_cast<std::uint32_t>(encoding[cursor] - '0');
        if (bit_depth >
            (std::numeric_limits<std::uint32_t>::max() - digit) / 10) {
            return false;
        }
        bit_depth = bit_depth * 10 + digit;
        ++cursor;
    }
    if (bit_depth != 8 && bit_depth != 16 && bit_depth != 32 &&
        bit_depth != 64) {
        return false;
    }
    if (cursor >= encoding.size() ||
        (encoding[cursor] != 'U' && encoding[cursor] != 'S' &&
         encoding[cursor] != 'F')) {
        return false;
    }
    const char scalar_kind = encoding[cursor++];
    const bool valid_scalar =
        (scalar_kind == 'U' && (bit_depth == 8 || bit_depth == 16)) ||
        (scalar_kind == 'S' &&
         (bit_depth == 8 || bit_depth == 16 || bit_depth == 32)) ||
        (scalar_kind == 'F' && (bit_depth == 32 || bit_depth == 64));
    if (!valid_scalar) {
        return false;
    }
    if (cursor >= encoding.size() || encoding[cursor++] != 'C') {
        return false;
    }
    if (cursor >= encoding.size()) {
        return false;
    }
    std::size_t channels = 0;
    while (cursor < encoding.size() &&
           std::isdigit(static_cast<unsigned char>(encoding[cursor]))) {
        const auto digit = static_cast<std::size_t>(encoding[cursor] - '0');
        if (channels > (std::numeric_limits<std::size_t>::max() - digit) / 10) {
            return false;
        }
        channels = channels * 10 + digit;
        ++cursor;
    }
    if (cursor != encoding.size() || channels == 0 || channels > 512) {
        return false;
    }
    return CheckedMultiply(bit_depth / 8, channels, bytes_per_pixel);
}

bool BytesPerPixel(std::string_view encoding, std::size_t* bytes_per_pixel) {
    if (encoding == "mono8" || encoding == "bayer_rggb8" ||
        encoding == "bayer_bggr8" || encoding == "bayer_gbrg8" ||
        encoding == "bayer_grbg8") {
        *bytes_per_pixel = 1;
        return true;
    }
    if (encoding == "mono16" || encoding == "bayer_rggb16" ||
        encoding == "bayer_bggr16" || encoding == "bayer_gbrg16" ||
        encoding == "bayer_grbg16" || encoding == "yuv422" ||
        encoding == "yuv422_yuy2" || encoding == "uyvy" || encoding == "yuyv") {
        *bytes_per_pixel = 2;
        return true;
    }
    if (encoding == "rgb8" || encoding == "bgr8") {
        *bytes_per_pixel = 3;
        return true;
    }
    if (encoding == "rgba8" || encoding == "bgra8") {
        *bytes_per_pixel = 4;
        return true;
    }
    if (encoding == "rgb16" || encoding == "bgr16") {
        *bytes_per_pixel = 6;
        return true;
    }
    if (encoding == "rgba16" || encoding == "bgra16") {
        *bytes_per_pixel = 8;
        return true;
    }
    return ParseGenericEncoding(encoding, bytes_per_pixel);
}

}  // namespace

bool ValidateImageData(const ImageData& image, std::string* error) {
    if (image.width == 0 || image.height == 0) {
        return Fail(error, "image width and height must be nonzero");
    }
    std::size_t bytes_per_pixel = 0;
    if (!BytesPerPixel(image.encoding, &bytes_per_pixel)) {
        return Fail(error, "unsupported or malformed image encoding: " +
                               image.encoding);
    }
    std::size_t minimum_step = 0;
    if (!CheckedMultiply(static_cast<std::size_t>(image.width), bytes_per_pixel,
                         &minimum_step) ||
        minimum_step > std::numeric_limits<std::uint32_t>::max()) {
        return Fail(error, "image row size overflows uint32 step");
    }
    if (image.step < minimum_step) {
        return Fail(error, "image step is smaller than encoded row width");
    }
    std::size_t expected_size = 0;
    if (!CheckedMultiply(static_cast<std::size_t>(image.height),
                         static_cast<std::size_t>(image.step),
                         &expected_size)) {
        return Fail(error, "image payload size overflows size_t");
    }
    if (image.data.size() != expected_size) {
        return Fail(error, "image data size must equal height * step");
    }
    return true;
}

bool FromProto(const ::automsgs::msgs::sensor_msgs::Image& message,
               ImageData* image, std::string* error) {
    if (image == nullptr) {
        return Fail(error, "ImageData output must not be null");
    }
    if (!message.has_header() || !message.header().has_stamp()) {
        return Fail(error, "image message must contain header.stamp");
    }

    ImageData result;
    if (!TimestampFromProto(message.header().stamp(), &result.timestamp_ns,
                            error)) {
        return false;
    }
    result.frame_id = message.header().frame_id();
    result.height = message.height();
    result.width = message.width();
    result.encoding = message.encoding();
    result.is_bigendian = message.is_bigendian();
    result.step = message.step();
    result.data.assign(message.data().begin(), message.data().end());
    if (!ValidateImageData(result, error)) {
        return false;
    }
    *image = std::move(result);
    return true;
}

bool ToProto(const ImageData& image,
             ::automsgs::msgs::sensor_msgs::Image* message,
             std::string* error) {
    if (message == nullptr) {
        return Fail(error, "image message output must not be null");
    }
    if (!ValidateImageData(image, error)) {
        return false;
    }

    ::automsgs::msgs::sensor_msgs::Image result;
    auto* header = result.mutable_header();
    if (!TimestampToProto(image.timestamp_ns, header->mutable_stamp(), error)) {
        return false;
    }
    header->set_frame_id(image.frame_id);
    result.set_height(image.height);
    result.set_width(image.width);
    result.set_encoding(image.encoding);
    result.set_is_bigendian(image.is_bigendian);
    result.set_step(image.step);
    result.set_data(image.data.data(), image.data.size());
    message->Swap(&result);
    return true;
}

}  // namespace autonomy::localization::atlas
