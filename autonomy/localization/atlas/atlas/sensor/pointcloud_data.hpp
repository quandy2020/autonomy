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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_SENSOR_POINTCLOUD_DATA_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_SENSOR_POINTCLOUD_DATA_HPP_

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include <cstdint>
#include <string>
#include <vector>

namespace autonomy::localization::atlas {

/** @brief Numeric storage identifiers compatible with PointCloud2 fields. */
enum class PointFieldDataType : std::uint8_t {
    kInt8 = 1,
    kUint8 = 2,
    kInt16 = 3,
    kUint16 = 4,
    kInt32 = 5,
    kUint32 = 6,
    kFloat32 = 7,
    kFloat64 = 8,
};

/** @brief Description of one packed field in a point record. */
struct PointFieldData {
    std::string name;
    std::uint32_t offset = 0;
    PointFieldDataType datatype = PointFieldDataType::kFloat32;
    std::uint32_t count = 1;
};

/** @brief Lossless validated representation of AutoMsgs PointCloud2. */
struct PointCloudData {
    std::int64_t timestamp_ns = 0;
    std::string frame_id;
    std::uint32_t height = 0;
    std::uint32_t width = 0;
    std::vector<PointFieldData> fields;
    bool is_bigendian = false;
    std::uint32_t point_step = 0;
    std::uint32_t row_step = 0;
    std::vector<std::uint8_t> data;
    bool is_dense = false;
};

/** @brief Validates fields, byte order, strides and payload bounds. */
bool ValidatePointCloudData(const PointCloudData& cloud,
                            std::string* error = nullptr);

/**
 * @brief Converts PointCloud2 without interpreting or byte-swapping fields.
 * @return false without modifying @p cloud when validation fails.
 */
bool FromProto(const ::automsgs::msgs::sensor_msgs::PointCloud2& message,
               PointCloudData* cloud, std::string* error = nullptr);

/** @brief Converts a validated cloud without modifying output on failure. */
bool ToProto(const PointCloudData& cloud,
             ::automsgs::msgs::sensor_msgs::PointCloud2* message,
             std::string* error = nullptr);

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_SENSOR_POINTCLOUD_DATA_HPP_
