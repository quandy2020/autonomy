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

#include "sensor/point_cloud_data.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>

#include "sensor/point_cloud_fields.hpp"

namespace argus {
namespace sensor {
namespace {

using Field = ::automsgs::msgs::sensor_msgs::PointField;
using PointCloud2 = ::automsgs::msgs::sensor_msgs::PointCloud2;

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

void SetHeader(::automsgs::msgs::std_msgs::Header* header, double stamp_sec,
               const std::string& frame_id) {
  if (header == nullptr) {
    return;
  }
  SetStamp(header->mutable_stamp(), stamp_sec);
  header->set_frame_id(frame_id);
}

void AddField(PointCloud2* msg, const char* name, std::uint32_t offset,
              Field::DataType datatype) {
  auto* f = msg->add_fields();
  f->set_name(name);
  f->set_offset(offset);
  f->set_datatype(datatype);
  f->set_count(1);
}

template <typename PointT>
void EncodeXYZI(const LidarCloud<PointT>& cloud, PointCloud2* msg,
                double stamp_sec, const std::string& frame_id) {
  if (msg == nullptr) {
    return;
  }
  msg->Clear();
  constexpr std::uint32_t kStep = 4 * sizeof(float);
  if (cloud.size() > std::numeric_limits<std::uint32_t>::max() ||
      cloud.size() > std::numeric_limits<std::size_t>::max() / kStep ||
      cloud.size() > std::numeric_limits<std::uint32_t>::max() / kStep) {
    return;
  }
  SetHeader(msg->mutable_header(), stamp_sec, frame_id);
  AddField(msg, "x", 0, Field::FLOAT32);
  AddField(msg, "y", 4, Field::FLOAT32);
  AddField(msg, "z", 8, Field::FLOAT32);
  AddField(msg, "intensity", 12, Field::FLOAT32);
  msg->set_point_step(kStep);
  msg->set_height(1);
  msg->set_width(static_cast<std::uint32_t>(cloud.size()));
  msg->set_row_step(kStep * msg->width());
  msg->set_is_dense(true);
  std::string bytes(cloud.size() * kStep, '\0');
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const PointT& point = cloud.points[i];
    const float packed[4] = {point.x, point.y, point.z, point.intensity};
    std::memcpy(bytes.data() + i * kStep, packed, sizeof(packed));
  }
  msg->set_data(std::move(bytes));
}

void FillRelativeTimeMs(float* curvature_ms, const char* base,
                        const FieldOffsets& offsets, double header_sec) {
  if (curvature_ms == nullptr) {
    return;
  }
  // Unified relative time in `curvature` (milliseconds).
  if (offsets.time_uint32 >= 0) {
    *curvature_ms =
        static_cast<float>(ReadAt<std::uint32_t>(base, offsets.time_uint32)) /
        1e6f;
  } else if (offsets.time >= 0) {
    *curvature_ms = ReadAt<float>(base, offsets.time);
  } else if (offsets.timestamp >= 0) {
    const double ts = ReadAt<double>(base, offsets.timestamp);
    if (ts > 1e12) {
      *curvature_ms = static_cast<float>((ts * 1e-9 - header_sec) * 1000.0);
    } else if (ts > 1e8) {
      *curvature_ms = static_cast<float>((ts - header_sec) * 1000.0);
    } else {
      *curvature_ms = static_cast<float>(ts * 1000.0);
    }
  }
}

}  // namespace

void FromProto(const PointCloud2& msg, LidarCloud<PointXYZINormal>* cloud) {
  double header_sec = 0.0;
  if (msg.has_header() && msg.header().has_stamp()) {
    header_sec = static_cast<double>(msg.header().stamp().sec()) +
                 1e-9 * static_cast<double>(msg.header().stamp().nanosec());
  }
  DecodeWithFiller(msg, cloud,
                   [header_sec](PointXYZINormal& pt, const char* base,
                                const FieldOffsets& offsets) {
                     pt.intensity = ReadIntensity(base, offsets);
                     FillRelativeTimeMs(&pt.curvature, base, offsets,
                                        header_sec);
                   });
}

void FromProto(const PointCloud2& msg, LidarCloud<PointXYZI>* cloud) {
  // PointXYZI has no curvature; LIVO uses PointCloudXYZI (=PointXYZINormal).
  DecodeWithFiller(
      msg, cloud,
      [](PointXYZI& pt, const char* base, const FieldOffsets& offsets) {
        pt.intensity = ReadIntensity(base, offsets);
      });
}

void FromProto(const PointCloud2& msg, LidarCloud<PointXYZRGB>* cloud) {
  DecodeWithFiller(
      msg, cloud,
      [](PointXYZRGB& pt, const char* base, const FieldOffsets& offsets) {
        if (offsets.rgb >= 0) {
          const float rgb_f = ReadAt<float>(base, offsets.rgb);
          std::uint32_t rgb = 0;
          std::memcpy(&rgb, &rgb_f, sizeof(rgb));
          pt.r = static_cast<std::uint8_t>((rgb >> 16) & 0xFF);
          pt.g = static_cast<std::uint8_t>((rgb >> 8) & 0xFF);
          pt.b = static_cast<std::uint8_t>(rgb & 0xFF);
        } else {
          pt.r = ReadAt<std::uint8_t>(base, offsets.r);
          pt.g = ReadAt<std::uint8_t>(base, offsets.g);
          pt.b = ReadAt<std::uint8_t>(base, offsets.b);
        }
      });
}

void ToProto(const LidarCloud<PointXYZINormal>& cloud, PointCloud2* msg,
             double stamp_sec, const std::string& frame_id) {
  EncodeXYZI(cloud, msg, stamp_sec, frame_id);
}

void ToProto(const LidarCloud<PointXYZI>& cloud, PointCloud2* msg,
             double stamp_sec, const std::string& frame_id) {
  EncodeXYZI(cloud, msg, stamp_sec, frame_id);
}

void ToProto(const LidarCloud<PointXYZRGB>& cloud, PointCloud2* msg,
             double stamp_sec, const std::string& frame_id) {
  if (msg == nullptr) {
    return;
  }
  msg->Clear();
  constexpr std::uint32_t kStep = 4 * sizeof(float);
  if (cloud.size() > std::numeric_limits<std::uint32_t>::max() / kStep ||
      cloud.size() > std::numeric_limits<std::size_t>::max() / kStep) {
    return;
  }
  SetHeader(msg->mutable_header(), stamp_sec, frame_id);
  AddField(msg, "x", 0, Field::FLOAT32);
  AddField(msg, "y", 4, Field::FLOAT32);
  AddField(msg, "z", 8, Field::FLOAT32);
  AddField(msg, "rgb", 12, Field::FLOAT32);
  msg->set_point_step(kStep);
  msg->set_height(1);
  msg->set_width(static_cast<std::uint32_t>(cloud.size()));
  msg->set_row_step(kStep * msg->width());
  msg->set_is_dense(true);
  std::string bytes(cloud.size() * kStep, '\0');
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const auto& p = cloud.points[i];
    const float xyz[3] = {p.x, p.y, p.z};
    const std::uint32_t rgb = (static_cast<std::uint32_t>(p.r) << 16) |
                              (static_cast<std::uint32_t>(p.g) << 8) |
                              static_cast<std::uint32_t>(p.b);
    float rgb_f = 0.f;
    std::memcpy(&rgb_f, &rgb, sizeof(float));
    std::memcpy(bytes.data() + i * kStep, xyz, sizeof(xyz));
    std::memcpy(bytes.data() + i * kStep + 12, &rgb_f, sizeof(float));
  }
  msg->set_data(std::move(bytes));
}

PointCloud2 ToProto(const LidarCloud<PointXYZINormal>& cloud, double stamp_sec,
                    const std::string& frame_id) {
  PointCloud2 msg;
  ToProto(cloud, &msg, stamp_sec, frame_id);
  return msg;
}

PointCloud2 ToProto(const LidarCloud<PointXYZRGB>& cloud, double stamp_sec,
                    const std::string& frame_id) {
  PointCloud2 msg;
  ToProto(cloud, &msg, stamp_sec, frame_id);
  return msg;
}

PointCloud2 ToProto(const LidarCloud<PointXYZI>& cloud, double stamp_sec,
                    const std::string& frame_id) {
  PointCloud2 msg;
  ToProto(cloud, &msg, stamp_sec, frame_id);
  return msg;
}

TimedPointCloudData ToTimedPointCloudData(const PointCloudXYZI& cloud,
                                          double time,
                                          const Eigen::Vector3f& origin) {
  TimedPointCloudData data;
  data.time = time;
  data.origin = origin;
  data.ranges.reserve(cloud.size());
  data.intensities.reserve(cloud.size());
  float latest = 0.f;
  for (const PointXYZINormal& point : cloud.points) {
    latest = std::max(latest, point.curvature);
  }
  for (const PointXYZINormal& point : cloud.points) {
    data.ranges.push_back(TimedRangefinderPoint{
        Eigen::Vector3f(point.x, point.y, point.z), point.curvature - latest});
    data.intensities.push_back(point.intensity);
  }
  return data;
}

}  // namespace sensor
}  // namespace argus
