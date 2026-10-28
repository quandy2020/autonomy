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

#include "atlas/sensor/pointcloud_data.hpp"

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/sensor_msgs/point_field.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>

namespace autonomy::localization::atlas {
namespace {

using ProtoField = ::automsgs::msgs::sensor_msgs::PointField;

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

bool CheckedAdd(std::size_t lhs, std::size_t rhs, std::size_t* result) {
    if (rhs > std::numeric_limits<std::size_t>::max() - lhs) {
        return false;
    }
    *result = lhs + rhs;
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

std::size_t PointFieldTypeSize(PointFieldDataType datatype) {
    switch (datatype) {
        case PointFieldDataType::kInt8:
        case PointFieldDataType::kUint8:
            return 1;
        case PointFieldDataType::kInt16:
        case PointFieldDataType::kUint16:
            return 2;
        case PointFieldDataType::kInt32:
        case PointFieldDataType::kUint32:
        case PointFieldDataType::kFloat32:
            return 4;
        case PointFieldDataType::kFloat64:
            return 8;
    }
    return 0;
}

bool DatatypeFromProto(ProtoField::DataType datatype,
                       PointFieldDataType* result) {
    switch (datatype) {
        case ProtoField::INT8:
            *result = PointFieldDataType::kInt8;
            return true;
        case ProtoField::UINT8:
            *result = PointFieldDataType::kUint8;
            return true;
        case ProtoField::INT16:
            *result = PointFieldDataType::kInt16;
            return true;
        case ProtoField::UINT16:
            *result = PointFieldDataType::kUint16;
            return true;
        case ProtoField::INT32:
            *result = PointFieldDataType::kInt32;
            return true;
        case ProtoField::UINT32:
            *result = PointFieldDataType::kUint32;
            return true;
        case ProtoField::FLOAT32:
            *result = PointFieldDataType::kFloat32;
            return true;
        case ProtoField::FLOAT64:
            *result = PointFieldDataType::kFloat64;
            return true;
        case ProtoField::DATA_TYPE_UNKNOWN:
            return false;
        default:
            return false;
    }
}

ProtoField::DataType DatatypeToProto(PointFieldDataType datatype) {
    switch (datatype) {
        case PointFieldDataType::kInt8:
            return ProtoField::INT8;
        case PointFieldDataType::kUint8:
            return ProtoField::UINT8;
        case PointFieldDataType::kInt16:
            return ProtoField::INT16;
        case PointFieldDataType::kUint16:
            return ProtoField::UINT16;
        case PointFieldDataType::kInt32:
            return ProtoField::INT32;
        case PointFieldDataType::kUint32:
            return ProtoField::UINT32;
        case PointFieldDataType::kFloat32:
            return ProtoField::FLOAT32;
        case PointFieldDataType::kFloat64:
            return ProtoField::FLOAT64;
    }
    return ProtoField::DATA_TYPE_UNKNOWN;
}

}  // namespace

bool ValidatePointCloudData(const PointCloudData& cloud, std::string* error) {
    if (cloud.height == 0 && cloud.width != 0) {
        return Fail(error,
                    "point cloud width must be zero when height is zero");
    }
    if (cloud.width == 0) {
        if (cloud.row_step != 0 || !cloud.data.empty()) {
            return Fail(error,
                        "empty point cloud must have zero row_step and data");
        }
    } else if (cloud.point_step == 0) {
        return Fail(error, "nonempty point cloud must have nonzero point_step");
    }
    if (cloud.width != 0 && cloud.fields.empty()) {
        return Fail(error,
                    "nonempty point cloud must describe at least one field");
    }

    std::size_t minimum_row_step = 0;
    if (!CheckedMultiply(static_cast<std::size_t>(cloud.width),
                         static_cast<std::size_t>(cloud.point_step),
                         &minimum_row_step) ||
        minimum_row_step > std::numeric_limits<std::uint32_t>::max()) {
        return Fail(error, "width * point_step overflows uint32 row_step");
    }
    if (cloud.row_step < minimum_row_step) {
        return Fail(error, "row_step is smaller than width * point_step");
    }

    std::size_t expected_size = 0;
    if (!CheckedMultiply(static_cast<std::size_t>(cloud.height),
                         static_cast<std::size_t>(cloud.row_step),
                         &expected_size)) {
        return Fail(error, "height * row_step overflows size_t");
    }
    if (cloud.data.size() != expected_size) {
        return Fail(error,
                    "point cloud data size must equal height * row_step");
    }

    std::unordered_set<std::string> field_names;
    for (const PointFieldData& field : cloud.fields) {
        if (field.name.empty()) {
            return Fail(error, "point field name must not be empty");
        }
        if (!field_names.insert(field.name).second) {
            return Fail(error, "duplicate point field name: " + field.name);
        }
        if (field.count == 0) {
            return Fail(error,
                        "point field count must be nonzero: " + field.name);
        }
        const std::size_t type_size = PointFieldTypeSize(field.datatype);
        if (type_size == 0) {
            return Fail(error, "point field has an unsupported datatype: " +
                                   field.name);
        }
        std::size_t field_size = 0;
        std::size_t field_end = 0;
        if (!CheckedMultiply(type_size, static_cast<std::size_t>(field.count),
                             &field_size) ||
            !CheckedAdd(static_cast<std::size_t>(field.offset), field_size,
                        &field_end) ||
            field_end > cloud.point_step) {
            return Fail(error, "point field exceeds point_step: " + field.name);
        }
    }
    return true;
}

bool FromProto(const ::automsgs::msgs::sensor_msgs::PointCloud2& message,
               PointCloudData* cloud, std::string* error) {
    if (cloud == nullptr) {
        return Fail(error, "PointCloudData output must not be null");
    }
    if (!message.has_header() || !message.header().has_stamp()) {
        return Fail(error, "point cloud message must contain header.stamp");
    }

    PointCloudData result;
    if (!TimestampFromProto(message.header().stamp(), &result.timestamp_ns,
                            error)) {
        return false;
    }
    result.frame_id = message.header().frame_id();
    result.height = message.height();
    result.width = message.width();
    result.is_bigendian = message.is_bigendian();
    result.point_step = message.point_step();
    result.row_step = message.row_step();
    result.is_dense = message.is_dense();
    result.data.assign(message.data().begin(), message.data().end());
    result.fields.reserve(static_cast<std::size_t>(message.fields_size()));
    for (const ProtoField& proto_field : message.fields()) {
        PointFieldData field;
        field.name = proto_field.name();
        field.offset = proto_field.offset();
        field.count = proto_field.count();
        if (!DatatypeFromProto(proto_field.datatype(), &field.datatype)) {
            return Fail(error, "point field has an unsupported datatype: " +
                                   field.name);
        }
        result.fields.push_back(std::move(field));
    }
    if (!ValidatePointCloudData(result, error)) {
        return false;
    }
    *cloud = std::move(result);
    return true;
}

bool ToProto(const PointCloudData& cloud,
             ::automsgs::msgs::sensor_msgs::PointCloud2* message,
             std::string* error) {
    if (message == nullptr) {
        return Fail(error, "point cloud message output must not be null");
    }
    if (!ValidatePointCloudData(cloud, error)) {
        return false;
    }

    ::automsgs::msgs::sensor_msgs::PointCloud2 result;
    auto* header = result.mutable_header();
    if (!TimestampToProto(cloud.timestamp_ns, header->mutable_stamp(), error)) {
        return false;
    }
    header->set_frame_id(cloud.frame_id);
    result.set_height(cloud.height);
    result.set_width(cloud.width);
    for (const PointFieldData& field : cloud.fields) {
        ProtoField* proto_field = result.add_fields();
        proto_field->set_name(field.name);
        proto_field->set_offset(field.offset);
        proto_field->set_datatype(DatatypeToProto(field.datatype));
        proto_field->set_count(field.count);
    }
    result.set_is_bigendian(cloud.is_bigendian);
    result.set_point_step(cloud.point_step);
    result.set_row_step(cloud.row_step);
    if (!cloud.data.empty()) {
        result.set_data(cloud.data.data(), cloud.data.size());
    }
    result.set_is_dense(cloud.is_dense);
    message->Swap(&result);
    return true;
}

}  // namespace autonomy::localization::atlas
