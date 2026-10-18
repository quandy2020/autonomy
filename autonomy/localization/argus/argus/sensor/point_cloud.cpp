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

#include "argus/sensor/point_cloud.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "argus/sensor/point_cloud_fields.hpp"

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

void EncodeXYZI(const float* xyz_i, std::size_t n, PointCloud2* msg,
                double stamp_sec, const std::string& frame_id) {
    msg->Clear();
    SetHeader(msg->mutable_header(), stamp_sec, frame_id);
    AddField(msg, "x", 0, Field::FLOAT32);
    AddField(msg, "y", 4, Field::FLOAT32);
    AddField(msg, "z", 8, Field::FLOAT32);
    AddField(msg, "intensity", 12, Field::FLOAT32);
    constexpr std::uint32_t step = 16;
    msg->set_point_step(step);
    msg->set_height(1);
    msg->set_width(static_cast<std::uint32_t>(n));
    msg->set_row_step(step * msg->width());
    msg->set_is_dense(true);
    std::string bytes(n * step, '\0');
    for (std::size_t i = 0; i < n; ++i) {
        std::memcpy(bytes.data() + i * step, xyz_i + i * 4, step);
    }
    msg->set_data(std::move(bytes));
}

}  // namespace

void FromProto(const PointCloud2& msg, PointCloud<PointXYZINormal>* cloud) {
    double header_sec = 0.0;
    if (msg.has_header() && msg.header().has_stamp()) {
        header_sec = static_cast<double>(msg.header().stamp().sec()) +
                     1e-9 * static_cast<double>(msg.header().stamp().nanosec());
    }
    DecodeWithFiller(
        msg, cloud,
        [header_sec](PointXYZINormal& pt, const char* base,
                     const FieldOffsets& offsets) {
            pt.intensity = ReadAt<float>(base, offsets.intensity);
            // Unified relative time in `curvature` (milliseconds).
            if (offsets.time_uint32 >= 0) {
                pt.curvature = static_cast<float>(
                                   ReadAt<std::uint32_t>(base,
                                                         offsets.time_uint32)) /
                               1e6f;
            } else if (offsets.time >= 0) {
                pt.curvature = ReadAt<float>(base, offsets.time);
            } else if (offsets.timestamp >= 0) {
                const double ts = ReadAt<double>(base, offsets.timestamp);
                if (ts > 1e12) {
                    // Nanoseconds (absolute or large offset).
                    pt.curvature =
                        static_cast<float>((ts * 1e-9 - header_sec) * 1000.0);
                } else if (ts > 1e8) {
                    // Absolute unix seconds.
                    pt.curvature =
                        static_cast<float>((ts - header_sec) * 1000.0);
                } else {
                    // Already relative seconds.
                    pt.curvature = static_cast<float>(ts * 1000.0);
                }
            }
        });
}

void FromProto(const PointCloud2& msg, PointCloud<PointXYZI>* cloud) {
    DecodeWithFiller(
        msg, cloud,
        [](PointXYZI& pt, const char* base, const FieldOffsets& offsets) {
            pt.intensity = ReadAt<float>(base, offsets.intensity);
        });
}

void FromProto(const PointCloud2& msg, PointCloud<PointXYZRGB>* cloud) {
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

void ToProto(const PointCloud<PointXYZINormal>& cloud, PointCloud2* msg,
             double stamp_sec, const std::string& frame_id) {
    std::vector<float> packed(cloud.size() * 4);
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        packed[i * 4 + 0] = cloud.points[i].x;
        packed[i * 4 + 1] = cloud.points[i].y;
        packed[i * 4 + 2] = cloud.points[i].z;
        packed[i * 4 + 3] = cloud.points[i].intensity;
    }
    EncodeXYZI(packed.data(), cloud.size(), msg, stamp_sec, frame_id);
}

void ToProto(const PointCloud<PointXYZI>& cloud, PointCloud2* msg,
             double stamp_sec, const std::string& frame_id) {
    std::vector<float> packed(cloud.size() * 4);
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        packed[i * 4 + 0] = cloud.points[i].x;
        packed[i * 4 + 1] = cloud.points[i].y;
        packed[i * 4 + 2] = cloud.points[i].z;
        packed[i * 4 + 3] = cloud.points[i].intensity;
    }
    EncodeXYZI(packed.data(), cloud.size(), msg, stamp_sec, frame_id);
}

void ToProto(const PointCloud<PointXYZRGB>& cloud, PointCloud2* msg,
             double stamp_sec, const std::string& frame_id) {
    msg->Clear();
    SetHeader(msg->mutable_header(), stamp_sec, frame_id);
    AddField(msg, "x", 0, Field::FLOAT32);
    AddField(msg, "y", 4, Field::FLOAT32);
    AddField(msg, "z", 8, Field::FLOAT32);
    AddField(msg, "rgb", 12, Field::FLOAT32);
    constexpr std::uint32_t step = 16;
    msg->set_point_step(step);
    msg->set_height(1);
    msg->set_width(static_cast<std::uint32_t>(cloud.size()));
    msg->set_row_step(step * msg->width());
    msg->set_is_dense(true);
    std::string bytes(cloud.size() * step, '\0');
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const auto& p = cloud.points[i];
        const float xyz[3] = {p.x, p.y, p.z};
        const std::uint32_t rgb = (static_cast<std::uint32_t>(p.r) << 16) |
                                  (static_cast<std::uint32_t>(p.g) << 8) |
                                  static_cast<std::uint32_t>(p.b);
        float rgb_f = 0.f;
        std::memcpy(&rgb_f, &rgb, sizeof(float));
        std::memcpy(bytes.data() + i * step, xyz, sizeof(xyz));
        std::memcpy(bytes.data() + i * step + 12, &rgb_f, sizeof(float));
    }
    msg->set_data(std::move(bytes));
}

PointCloud2 ToProto(const PointCloud<PointXYZINormal>& cloud, double stamp_sec,
                    const std::string& frame_id) {
    PointCloud2 msg;
    ToProto(cloud, &msg, stamp_sec, frame_id);
    return msg;
}

PointCloud2 ToProto(const PointCloud<PointXYZRGB>& cloud, double stamp_sec,
                    const std::string& frame_id) {
    PointCloud2 msg;
    ToProto(cloud, &msg, stamp_sec, frame_id);
    return msg;
}

PointCloud2 ToProto(const PointCloud<PointXYZI>& cloud, double stamp_sec,
                    const std::string& frame_id) {
    PointCloud2 msg;
    ToProto(cloud, &msg, stamp_sec, frame_id);
    return msg;
}

}  // namespace sensor
}  // namespace argus
