/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/observation/point_cloud_encoder.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <automsgs/msgs/sensor_msgs/point_field.pb.h>

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

bool FindXyzOffsets(const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
                    int* x_offset, int* y_offset, int* z_offset) {
    if (x_offset == nullptr || y_offset == nullptr || z_offset == nullptr) {
        return false;
    }
    *x_offset = -1;
    *y_offset = -1;
    *z_offset = -1;
    for (int i = 0; i < cloud.fields_size(); ++i) {
        const auto& field = cloud.fields(i);
        if (field.datatype() !=
            automsgs::msgs::sensor_msgs::PointField::FLOAT32) {
            continue;
        }
        if (field.name() == "x") {
            *x_offset = static_cast<int>(field.offset());
        } else if (field.name() == "y") {
            *y_offset = static_cast<int>(field.offset());
        } else if (field.name() == "z") {
            *z_offset = static_cast<int>(field.offset());
        }
    }
    return *x_offset >= 0 && *y_offset >= 0 && *z_offset >= 0;
}

float ReadFloat32(const std::string& data, std::size_t absolute_offset,
                  bool bigendian) {
    if (absolute_offset + sizeof(float) > data.size()) {
        return 0.0f;
    }
    float value = 0.0f;
    if (!bigendian) {
        std::memcpy(&value, data.data() + absolute_offset, sizeof(float));
        return value;
    }
    unsigned char bytes[4];
    std::memcpy(bytes, data.data() + absolute_offset, 4);
    std::swap(bytes[0], bytes[3]);
    std::swap(bytes[1], bytes[2]);
    std::memcpy(&value, bytes, sizeof(float));
    return value;
}

int ResolvePointCount(const proto::ObservationInput& input, int max_points) {
    int count = std::max(1, max_points);
    if (input.shape_size() == 0) {
        return count;
    }
    // Prefer [1, N, 3] or [N, 3] → N; else product / 3.
    if (input.shape_size() >= 2) {
        const int last = static_cast<int>(input.shape(input.shape_size() - 1));
        const int mid = static_cast<int>(input.shape(input.shape_size() - 2));
        if (last == 3 && mid > 0) {
            return mid;
        }
    }
    int64_t product = 1;
    bool any = false;
    for (int i = 0; i < input.shape_size(); ++i) {
        if (input.shape(i) > 0) {
            product *= input.shape(i);
            any = true;
        }
    }
    if (any && product >= 3) {
        return static_cast<int>(product / 3);
    }
    return count;
}

}  // namespace

bool EncodePointCloud(const ObservationSnapshot& snapshot,
                      const proto::ObservationInput& input, int max_points,
                      std::vector<float>* output, std::string* error,
                      bool stride_sample) {
    if (output == nullptr) {
        return false;
    }
    if (snapshot.point_cloud == nullptr ||
        snapshot.point_cloud->data().empty()) {
        if (error != nullptr) {
            *error = "Point-cloud modality requires PointCloud2 data";
        }
        return false;
    }

    const auto& cloud = *snapshot.point_cloud;
    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    if (!FindXyzOffsets(cloud, &x_offset, &y_offset, &z_offset)) {
        if (error != nullptr) {
            *error = "PointCloud2 missing FLOAT32 x/y/z fields";
        }
        return false;
    }

    const int point_step = static_cast<int>(cloud.point_step());
    if (point_step <= 0) {
        if (error != nullptr) {
            *error = "PointCloud2 point_step is invalid";
        }
        return false;
    }

    const int source_points = static_cast<int>(cloud.height() * cloud.width());
    const int target_points = ResolvePointCount(input, max_points);
    output->assign(static_cast<std::size_t>(target_points * 3), 0.0f);

    const std::string& bytes = cloud.data();
    const int stride =
        stride_sample && source_points > target_points
            ? std::max(1, source_points / target_points)
            : 1;
    const int copy_points =
        stride_sample ? std::min(target_points, (source_points + stride - 1) / stride)
                      : std::min(source_points, target_points);
    for (int i = 0; i < copy_points; ++i) {
        const int source_index = stride_sample ? i * stride : i;
        if (source_index >= source_points) {
            break;
        }
        const std::size_t base = static_cast<std::size_t>(source_index) *
                                 static_cast<std::size_t>(point_step);
        (*output)[static_cast<std::size_t>(i * 3 + 0)] =
            ReadFloat32(bytes, base + static_cast<std::size_t>(x_offset),
                        cloud.is_bigendian());
        (*output)[static_cast<std::size_t>(i * 3 + 1)] =
            ReadFloat32(bytes, base + static_cast<std::size_t>(y_offset),
                        cloud.is_bigendian());
        (*output)[static_cast<std::size_t>(i * 3 + 2)] =
            ReadFloat32(bytes, base + static_cast<std::size_t>(z_offset),
                        cloud.is_bigendian());
    }
    return true;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
