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

#include "sensor/msg_conversion.hpp"

#include <automsgs/msgs/builtin_interfaces/time.pb.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include "sensor/point_cloud_fields.hpp"

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

float ReadRelativeTime(const char* base, const FieldOffsets& offsets) {
  if (offsets.time >= 0) {
    return ReadAt<float>(base, offsets.time);
  }
  if (offsets.time_uint32 >= 0) {
    return static_cast<float>(ReadAt<uint32_t>(base, offsets.time_uint32)) *
           1e-9f;
  }
  if (offsets.timestamp >= 0) {
    return static_cast<float>(ReadAt<double>(base, offsets.timestamp));
  }
  return 0.f;
}

void ShiftToLastReturn(TimedPointCloudData* cloud) {
  if (cloud->ranges.empty()) {
    return;
  }
  const float duration = cloud->ranges.back().time;
  cloud->time += static_cast<double>(duration);
  for (TimedRangefinderPoint& point : cloud->ranges) {
    point.time -= duration;
  }
}

void AddField(PointCloud2* msg, const char* name, uint32_t offset) {
  auto* field = msg->add_fields();
  field->set_name(name);
  field->set_offset(offset);
  field->set_datatype(PointField::FLOAT32);
  field->set_count(1);
}

}  // namespace

TimedPointCloudData FromProto(
    const ::automsgs::msgs::sensor_msgs::PointCloud2& msg) {
  TimedPointCloudData cloud;
  if (msg.has_header() && msg.header().has_stamp()) {
    cloud.time = StampToSec(msg.header().stamp());
  }
  const FieldOffsets offsets = ParseFields(msg);
  if (!IsPointCloud2LayoutValid(msg, offsets)) {
    return cloud;
  }
  const auto step = static_cast<std::size_t>(msg.point_step());
  const auto width = static_cast<std::size_t>(msg.width());
  const auto height = static_cast<std::size_t>(msg.height());
  const auto row_step = static_cast<std::size_t>(msg.row_step());
  const std::string& data = msg.data();
  for (std::size_t row = 0; row < height; ++row) {
    for (std::size_t column = 0; column < width; ++column) {
      const char* base = data.data() + row * row_step + column * step;
      const float x = ReadAt<float>(base, offsets.x);
      const float y = ReadAt<float>(base, offsets.y);
      const float z = ReadAt<float>(base, offsets.z);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      cloud.ranges.push_back(TimedRangefinderPoint{
          Eigen::Vector3f(x, y, z), ReadRelativeTime(base, offsets)});
      cloud.intensities.push_back(
          offsets.intensity >= 0 ? ReadIntensity(base, offsets) : 1.f);
    }
  }
  ShiftToLastReturn(&cloud);
  return cloud;
}

::automsgs::msgs::sensor_msgs::PointCloud2 ToProto(
    const TimedPointCloudData& cloud, const std::string& frame_id) {
  ::automsgs::msgs::sensor_msgs::PointCloud2 msg;
  SetStamp(msg.mutable_header()->mutable_stamp(), cloud.time);
  msg.mutable_header()->set_frame_id(frame_id);
  constexpr uint32_t kStep = 20;
  AddField(&msg, "x", 0);
  AddField(&msg, "y", 4);
  AddField(&msg, "z", 8);
  AddField(&msg, "intensity", 12);
  AddField(&msg, "time", 16);
  msg.set_height(1);
  msg.set_width(static_cast<uint32_t>(cloud.ranges.size()));
  msg.set_is_bigendian(false);
  msg.set_point_step(kStep);
  msg.set_row_step(kStep * msg.width());
  msg.set_is_dense(true);
  std::string bytes(static_cast<std::size_t>(msg.row_step()), '\0');
  for (std::size_t i = 0; i < cloud.ranges.size(); ++i) {
    char* base = bytes.data() + i * kStep;
    const Eigen::Vector3f& p = cloud.ranges[i].position;
    const float xyz[3] = {p.x(), p.y(), p.z()};
    std::memcpy(base, xyz, sizeof(xyz));
    const float intensity =
        i < cloud.intensities.size() ? cloud.intensities[i] : 1.f;
    std::memcpy(base + 12, &intensity, sizeof(float));
    std::memcpy(base + 16, &cloud.ranges[i].time, sizeof(float));
  }
  msg.set_data(std::move(bytes));
  return msg;
}

TimedPointCloudData FromProto(
    const ::automsgs::msgs::sensor_msgs::LaserScan& msg) {
  TimedPointCloudData cloud;
  if (msg.has_header() && msg.header().has_stamp()) {
    cloud.time = StampToSec(msg.header().stamp());
  }
  if (msg.range_min() < 0.f || msg.range_max() < msg.range_min()) {
    return cloud;
  }
  float angle = msg.angle_min();
  const bool has_intensity = msg.intensities_size() == msg.ranges_size();
  for (int i = 0; i < msg.ranges_size(); ++i) {
    const float range = msg.ranges(i);
    if (std::isfinite(range) && msg.range_min() <= range &&
        range <= msg.range_max()) {
      const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
      cloud.ranges.push_back(
          TimedRangefinderPoint{rotation * (range * Eigen::Vector3f::UnitX()),
                                static_cast<float>(i) * msg.time_increment()});
      cloud.intensities.push_back(has_intensity ? msg.intensities(i) : 0.f);
    }
    angle += msg.angle_increment();
  }
  ShiftToLastReturn(&cloud);
  return cloud;
}

PointCloud PointCloudFromProto(
    const ::automsgs::msgs::sensor_msgs::PointCloud2& msg) {
  const TimedPointCloudData timed = FromProto(msg);
  std::vector<RangefinderPoint> points;
  points.reserve(timed.ranges.size());
  for (const TimedRangefinderPoint& point : timed.ranges) {
    points.push_back(ToRangefinderPoint(point));
  }
  return PointCloud(std::move(points), timed.intensities);
}

}  // namespace sensor
}  // namespace argus
