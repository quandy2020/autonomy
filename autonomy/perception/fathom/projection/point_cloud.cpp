/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file point_cloud.cpp
 * @brief Camera-model projection into organized XYZ PointCloud2 messages.
 */

#include "autonomy/perception/fathom/projection/point_cloud.hpp"

#include <automsgs/msgs/sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>

namespace autonomy {
namespace perception {
namespace fathom {

namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

bool ValidateImage(const automsgs::msgs::sensor_msgs::Image& image,
                   const char* encoding, uint32_t bytes_per_pixel,
                   std::string* error) {
    if (image.width() == 0 || image.height() == 0) {
        SetError(error, "Fathom projection image dimensions must be positive.");
        return false;
    }
    if (image.encoding() != encoding || image.is_bigendian()) {
        SetError(error, "Fathom projection image encoding is unsupported.");
        return false;
    }
    const size_t minimum_step =
        static_cast<size_t>(image.width()) * bytes_per_pixel;
    if (image.step() < minimum_step ||
        image.data().size() <
            static_cast<size_t>(image.height()) * image.step()) {
        SetError(error, "Fathom projection image step or data is invalid.");
        return false;
    }
    return true;
}

}  // namespace

bool ProjectDepth(const automsgs::msgs::sensor_msgs::Image& depth_m,
                  const automsgs::msgs::sensor_msgs::Image& mask,
                  const automsgs::msgs::sensor_msgs::CameraInfo& camera_info,
                  automsgs::msgs::sensor_msgs::PointCloud2* cloud,
                  std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (cloud != nullptr) {
        cloud->Clear();
    }
    if (cloud == nullptr) {
        SetError(error, "Fathom point cloud output is null.");
        return false;
    }
    if (!ValidateImage(depth_m, "32FC1", 4, error) ||
        !ValidateImage(mask, "mono8", 1, error)) {
        return false;
    }
    if (depth_m.width() != mask.width() || depth_m.height() != mask.height()) {
        SetError(error,
                 "Fathom depth and validity mask dimensions must match.");
        return false;
    }
    if (camera_info.width() == 0 || camera_info.height() == 0 ||
        camera_info.width() != depth_m.width() ||
        camera_info.height() != depth_m.height()) {
        SetError(error,
                 "Fathom camera calibration dimensions must be positive and "
                 "match the depth and validity images.");
        return false;
    }
    if (camera_info.k_size() != 9) {
        SetError(error, "Fathom camera matrix must contain nine values.");
        return false;
    }
    const float fx = static_cast<float>(camera_info.k(0));
    const float fy = static_cast<float>(camera_info.k(4));
    const float cx = static_cast<float>(camera_info.k(2));
    const float cy = static_cast<float>(camera_info.k(5));
    if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) ||
        !std::isfinite(cy) || fx <= 0.0F || fy <= 0.0F) {
        SetError(error,
                 "Fathom camera intrinsics must be finite with positive focal "
                 "lengths.");
        return false;
    }

    automsgs::msgs::sensor_msgs::PointCloud2 projected;
    *projected.mutable_header() = depth_m.header();
    projected.set_height(depth_m.height());
    projected.set_width(depth_m.width());
    projected.set_is_bigendian(false);
    projected.set_is_dense(false);
    projected.set_point_step(3 * sizeof(float));
    projected.set_row_step(projected.width() * projected.point_step());
    for (const char* name : {"x", "y", "z"}) {
        auto* field = projected.add_fields();
        field->set_name(name);
        field->set_offset(static_cast<uint32_t>(projected.fields_size() - 1) *
                          sizeof(float));
        field->set_datatype(automsgs::msgs::sensor_msgs::PointField::FLOAT32);
        field->set_count(1);
    }
    projected.mutable_data()->resize(static_cast<size_t>(projected.height()) *
                                     projected.row_step());

    automsgs::msgs::sensor_msgs::PointCloud2Iterator<float> x(projected, "x");
    automsgs::msgs::sensor_msgs::PointCloud2Iterator<float> y(projected, "y");
    automsgs::msgs::sensor_msgs::PointCloud2Iterator<float> z(projected, "z");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (uint32_t row = 0; row < depth_m.height(); ++row) {
        for (uint32_t col = 0; col < depth_m.width(); ++col, ++x, ++y, ++z) {
            const size_t depth_offset =
                static_cast<size_t>(row) * depth_m.step() +
                static_cast<size_t>(col) * sizeof(float);
            float depth = 0.0F;
            std::memcpy(&depth, depth_m.data().data() + depth_offset,
                        sizeof(depth));
            const uint8_t validity = static_cast<uint8_t>(
                mask.data()[static_cast<size_t>(row) * mask.step() + col]);
            if (validity == 0 || !std::isfinite(depth) || depth <= 0.0F) {
                *x = nan;
                *y = nan;
                *z = nan;
            } else {
                *x = (static_cast<float>(col) - cx) * depth / fx;
                *y = (static_cast<float>(row) - cy) * depth / fy;
                *z = depth;
            }
        }
    }
    cloud->CopyFrom(projected);
    return true;
}

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy
