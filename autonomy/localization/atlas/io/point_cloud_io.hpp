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
 * @file point_cloud_io.hpp
 * @brief Decode automsgs PointCloud2 into Atlas points. Algorithms stay off proto.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_IO_POINT_CLOUD_IO_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_IO_POINT_CLOUD_IO_HPP_

#include <cstdint>
#include <cstring>
#include <string>

#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/sensor/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @brief Read x/y/z (and optional intensity) from a PointCloud2.
 * @param cloud Input message. FLOAT32 fields named x, y, z.
 * @param[out] out Appended points. Cleared by the caller.
 * @return false when the layout cannot be read.
 */
inline bool DecodePointCloud2(const PointCloud2& cloud, PointCloud* out) {
    if (out == nullptr || cloud.point_step() == 0 || cloud.data().empty()) {
        return false;
    }
    using Field = ::automsgs::msgs::sensor_msgs::PointField;
    int ox = -1;
    int oy = -1;
    int oz = -1;
    int oi = -1;
    int time_off = -1;
    int t_ns_off = -1;
    int timestamp_off = -1;
    for (const auto& field : cloud.fields()) {
        const int offset = static_cast<int>(field.offset());
        if (field.name() == "x" && field.datatype() == Field::FLOAT32) {
            ox = offset;
        } else if (field.name() == "y" && field.datatype() == Field::FLOAT32) {
            oy = offset;
        } else if (field.name() == "z" && field.datatype() == Field::FLOAT32) {
            oz = offset;
        } else if (field.name() == "intensity" &&
                   field.datatype() == Field::FLOAT32) {
            oi = offset;
        } else if (field.name() == "t" && field.datatype() == Field::UINT32) {
            t_ns_off = offset;
        } else if (field.name() == "time" && field.datatype() == Field::FLOAT32) {
            time_off = offset;
        } else if (field.name() == "timestamp" &&
                   field.datatype() == Field::FLOAT64) {
            timestamp_off = offset;
        }
    }
    if (ox < 0 || oy < 0 || oz < 0) {
        return false;
    }
    const auto step = static_cast<std::size_t>(cloud.point_step());
    const std::string& bytes = cloud.data();
    const double stamp = HeaderStampSec(cloud.header());
    for (std::size_t offset = 0; offset + step <= bytes.size(); offset += step) {
        auto ReadFloat = [&](int field_offset) {
            float value = 0.f;
            std::memcpy(&value, bytes.data() + offset +
                                    static_cast<std::size_t>(field_offset),
                        sizeof(float));
            return value;
        };
        PointXYZI point;
        point.x = ReadFloat(ox);
        point.y = ReadFloat(oy);
        point.z = ReadFloat(oz);
        point.intensity = oi >= 0 ? ReadFloat(oi) : 0.f;
        point.timestamp = stamp;
        if (t_ns_off >= 0) {
            uint32_t nanoseconds = 0;
            std::memcpy(&nanoseconds,
                        bytes.data() + offset +
                            static_cast<std::size_t>(t_ns_off),
                        sizeof(uint32_t));
            point.timestamp = stamp + static_cast<double>(nanoseconds) * 1e-9;
        } else if (time_off >= 0) {
            point.timestamp = stamp + static_cast<double>(ReadFloat(time_off));
        } else if (timestamp_off >= 0) {
            double absolute = stamp;
            std::memcpy(&absolute,
                        bytes.data() + offset +
                            static_cast<std::size_t>(timestamp_off),
                        sizeof(double));
            point.timestamp = absolute;
        }
        if (point.x == point.x && point.y == point.y && point.z == point.z) {
            out->push_back(point);
        }
    }
    return !out->empty();
}

/**
 * @brief Write x/y/z FLOAT32 into a PointCloud2.
 * @param cloud Source points.
 * @param[out] out Cleared and filled message.
 * @param frame `header.frame_id`, usually `map`.
 */
inline void EncodePointCloud2(const PointCloud& cloud, PointCloud2* out,
                              const std::string& frame) {
    if (out == nullptr) {
        return;
    }
    out->Clear();
    out->mutable_header()->set_frame_id(frame);
    using Field = ::automsgs::msgs::sensor_msgs::PointField;
    const char* names[] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
        auto* field = out->add_fields();
        field->set_name(names[i]);
        field->set_offset(static_cast<uint32_t>(i * static_cast<int>(sizeof(float))));
        field->set_datatype(Field::FLOAT32);
        field->set_count(1);
    }
    const auto step = static_cast<uint32_t>(3 * sizeof(float));
    out->set_point_step(step);
    out->set_height(1);
    out->set_width(static_cast<uint32_t>(cloud.size()));
    out->set_row_step(step * out->width());
    out->set_is_dense(true);
    std::string bytes(static_cast<std::size_t>(cloud.size()) * step, '\0');
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const float xyz[3] = {cloud[i].x, cloud[i].y, cloud[i].z};
        std::memcpy(bytes.data() + i * step, xyz, step);
    }
    out->set_data(std::move(bytes));
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_IO_POINT_CLOUD_IO_HPP_
