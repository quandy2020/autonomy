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

#include "sensor/image_data.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <opencv2/imgcodecs.hpp>
#include <vector>

namespace argus {
namespace sensor {
namespace {

double StampToSec(const ::automsgs::msgs::builtin_interfaces::Time& stamp) {
  return static_cast<double>(stamp.sec()) +
         1e-9 * static_cast<double>(stamp.nanosec());
}

void SetStamp(::automsgs::msgs::builtin_interfaces::Time* stamp,
              double timestamp_sec) {
  if (stamp == nullptr) {
    return;
  }
  auto sec = static_cast<int32_t>(timestamp_sec);
  auto nanosec = static_cast<uint32_t>(
      std::llround((timestamp_sec - static_cast<double>(sec)) * 1e9));
  if (nanosec >= 1000000000u) {
    stamp->set_sec(sec + 1);
    stamp->set_nanosec(nanosec - 1000000000u);
  } else {
    stamp->set_sec(sec);
    stamp->set_nanosec(nanosec);
  }
}

void ReadHeader(const ::automsgs::msgs::std_msgs::Header& header, double* time,
                std::string* frame_id) {
  if (header.has_stamp()) {
    *time = StampToSec(header.stamp());
  }
  *frame_id = header.frame_id();
}

}  // namespace

ImageData FromProto(const ::automsgs::msgs::sensor_msgs::Image& msg) {
  ImageData image;
  if (msg.has_header()) {
    ReadHeader(msg.header(), &image.time, &image.frame_id);
  }
  image.height = msg.height();
  image.width = msg.width();
  image.encoding = msg.encoding();
  image.step = msg.step();
  image.data.assign(msg.data().begin(), msg.data().end());
  return image;
}

::automsgs::msgs::sensor_msgs::Image ToProto(const ImageData& image) {
  ::automsgs::msgs::sensor_msgs::Image msg;
  SetStamp(msg.mutable_header()->mutable_stamp(), image.time);
  msg.mutable_header()->set_frame_id(image.frame_id);
  msg.set_height(image.height);
  msg.set_width(image.width);
  msg.set_encoding(image.encoding);
  msg.set_is_bigendian(false);
  msg.set_step(image.step);
  msg.set_data(image.data.data(), image.data.size());
  return msg;
}

CompressedImageData FromProto(
    const ::automsgs::msgs::sensor_msgs::CompressedImage& msg) {
  CompressedImageData image;
  if (msg.has_header()) {
    ReadHeader(msg.header(), &image.time, &image.frame_id);
  }
  image.format = msg.format();
  image.data.assign(msg.data().begin(), msg.data().end());
  return image;
}

::automsgs::msgs::sensor_msgs::CompressedImage ToProto(
    const CompressedImageData& image) {
  ::automsgs::msgs::sensor_msgs::CompressedImage msg;
  SetStamp(msg.mutable_header()->mutable_stamp(), image.time);
  msg.mutable_header()->set_frame_id(image.frame_id);
  msg.set_format(image.format);
  msg.set_data(image.data.data(), image.data.size());
  return msg;
}

bool DecodeCompressedImage(const CompressedImageData& compressed,
                           ImageData* out) {
  if (out == nullptr || compressed.data.empty()) {
    return false;
  }
  const std::vector<std::uint8_t> buffer = compressed.data;
  const cv::Mat decoded = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
  if (decoded.empty()) {
    return false;
  }
  ImageData image;
  image.time = compressed.time;
  image.frame_id = compressed.frame_id;
  image.height = static_cast<std::uint32_t>(decoded.rows);
  image.width = static_cast<std::uint32_t>(decoded.cols);
  image.step = static_cast<std::uint32_t>(decoded.step);
  if (decoded.channels() == 1) {
    image.encoding = "mono8";
  } else if (decoded.channels() == 3) {
    image.encoding = "bgr8";
  } else if (decoded.channels() == 4) {
    image.encoding = "bgra8";
  } else {
    return false;
  }
  image.data.resize(static_cast<std::size_t>(decoded.rows) *
                    static_cast<std::size_t>(decoded.step));
  for (int row = 0; row < decoded.rows; ++row) {
    std::memcpy(
        image.data.data() + static_cast<std::size_t>(row) * decoded.step,
        decoded.ptr(row), static_cast<std::size_t>(decoded.step));
  }
  *out = std::move(image);
  return true;
}

}  // namespace sensor
}  // namespace argus
