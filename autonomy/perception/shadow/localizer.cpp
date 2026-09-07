/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file localizer.cpp
 * @brief Robust depth, projection, transform, and velocity filtering.
 */

#include "autonomy/perception/shadow/localizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace autonomy {
namespace perception {
namespace shadow {
namespace {

using CameraInfo = automsgs::msgs::sensor_msgs::CameraInfo;
using Detection2D = automsgs::msgs::vision_msgs::Detection2D;
using Image = automsgs::msgs::sensor_msgs::Image;
using TransformStamped = automsgs::msgs::geometry_msgs::TransformStamped;

constexpr double kNanosecondsPerSecond = 1.0e9;
constexpr double kPositionFilterAlpha = 0.5;

struct image_region {
    uint32_t begin_x = 0;
    uint32_t end_x = 0;
    uint32_t begin_y = 0;
    uint32_t end_y = 0;
};

struct point_3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

void set_error(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Shadow localizer: " + message;
    }
}

bool finite_positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool validate_depth_options(const proto::ShadowOptions& options,
                            std::string* error) {
    if (!finite_positive(options.inner_box_scale()) ||
        options.inner_box_scale() > 1.0F ||
        !finite_positive(options.min_depth_m()) ||
        !finite_positive(options.max_depth_m()) ||
        options.max_depth_m() <= options.min_depth_m() ||
        options.min_depth_samples() == 0 ||
        !finite_positive(options.depth_outlier_m()) ||
        !finite_positive(options.depth_scale())) {
        set_error(error, "depth options are invalid.");
        return false;
    }
    return true;
}

bool validate_camera(const CameraInfo& camera, const Image& depth,
                     std::string* error) {
    if (camera.width() == 0 || camera.height() == 0 ||
        camera.width() != depth.width() || camera.height() != depth.height()) {
        set_error(error, "camera dimensions must be positive and match depth.");
        return false;
    }
    if (camera.k_size() < 9 || !finite_positive(camera.k(0)) ||
        !finite_positive(camera.k(4)) || !std::isfinite(camera.k(2)) ||
        !std::isfinite(camera.k(5))) {
        set_error(error, "camera intrinsics are invalid.");
        return false;
    }
    return true;
}

bool validate_depth_image(const Image& depth, uint32_t* bytes_per_pixel,
                          std::string* error) {
    if (depth.encoding() == "16UC1") {
        *bytes_per_pixel = sizeof(uint16_t);
    } else if (depth.encoding() == "32FC1") {
        *bytes_per_pixel = sizeof(float);
    } else {
        set_error(error, "depth encoding must be '16UC1' or '32FC1'.");
        return false;
    }
    if (depth.width() == 0 || depth.height() == 0) {
        set_error(error, "depth dimensions must be positive.");
        return false;
    }
    const size_t minimum_step =
        static_cast<size_t>(depth.width()) * *bytes_per_pixel;
    if (depth.step() < minimum_step) {
        set_error(error, "depth image step is too small.");
        return false;
    }
    if (depth.step() != 0 &&
        depth.height() > std::numeric_limits<size_t>::max() /
                             static_cast<size_t>(depth.step())) {
        set_error(error, "depth image layout exceeds addressable size.");
        return false;
    }
    const size_t required_size =
        static_cast<size_t>(depth.height()) * depth.step();
    if (depth.data().size() < required_size) {
        set_error(error, "depth image data is too small.");
        return false;
    }
    return true;
}

uint16_t decode_uint16(const char* data, bool big_endian) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(data);
    if (big_endian) {
        return static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8) |
                                     static_cast<uint16_t>(bytes[1]));
    }
    return static_cast<uint16_t>(static_cast<uint16_t>(bytes[0]) |
                                 (static_cast<uint16_t>(bytes[1]) << 8));
}

float decode_float32(const char* data, bool big_endian) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(data);
    uint32_t bits = 0;
    if (big_endian) {
        bits = (static_cast<uint32_t>(bytes[0]) << 24) |
               (static_cast<uint32_t>(bytes[1]) << 16) |
               (static_cast<uint32_t>(bytes[2]) << 8) |
               static_cast<uint32_t>(bytes[3]);
    } else {
        bits = static_cast<uint32_t>(bytes[0]) |
               (static_cast<uint32_t>(bytes[1]) << 8) |
               (static_cast<uint32_t>(bytes[2]) << 16) |
               (static_cast<uint32_t>(bytes[3]) << 24);
    }
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool inner_region(const Detection2D& detection, const Image& depth, float scale,
                  image_region* region, std::string* error) {
    const auto& box = detection.bbox();
    const double center_x = box.center().position().x();
    const double center_y = box.center().position().y();
    const double width = box.size_x();
    const double height = box.size_y();
    if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
        !finite_positive(width) || !finite_positive(height)) {
        set_error(error, "detection bounding box is invalid.");
        return false;
    }

    const double half_width = width * static_cast<double>(scale) * 0.5;
    const double half_height = height * static_cast<double>(scale) * 0.5;
    const double image_width = static_cast<double>(depth.width());
    const double image_height = static_cast<double>(depth.height());
    const double left = std::clamp(center_x - half_width, 0.0, image_width);
    const double right = std::clamp(center_x + half_width, 0.0, image_width);
    const double top = std::clamp(center_y - half_height, 0.0, image_height);
    const double bottom = std::clamp(center_y + half_height, 0.0, image_height);
    region->begin_x = static_cast<uint32_t>(std::ceil(left));
    region->end_x = static_cast<uint32_t>(std::ceil(right));
    region->begin_y = static_cast<uint32_t>(std::ceil(top));
    region->end_y = static_cast<uint32_t>(std::ceil(bottom));
    if (region->begin_x >= region->end_x || region->begin_y >= region->end_y) {
        set_error(error, "detection inner box contains no depth pixels.");
        return false;
    }
    return true;
}

float median(std::vector<float>* values) {
    std::sort(values->begin(), values->end());
    const size_t middle = values->size() / 2;
    if (values->size() % 2 != 0) {
        return (*values)[middle];
    }
    return (*values)[middle - 1] +
           ((*values)[middle] - (*values)[middle - 1]) * 0.5F;
}

bool split_stamp(int64_t stamp_ns, int32_t* seconds, uint32_t* nanoseconds,
                 std::string* error) {
    constexpr int64_t nanoseconds_per_second = 1'000'000'000;
    int64_t sec = stamp_ns / nanoseconds_per_second;
    int64_t nsec = stamp_ns % nanoseconds_per_second;
    if (nsec < 0) {
        --sec;
        nsec += nanoseconds_per_second;
    }
    if (sec < std::numeric_limits<int32_t>::min() ||
        sec > std::numeric_limits<int32_t>::max()) {
        set_error(error, "timestamp is outside automsgs Time range.");
        return false;
    }
    *seconds = static_cast<int32_t>(sec);
    *nanoseconds = static_cast<uint32_t>(nsec);
    return true;
}

bool validate_transform(const proto::ShadowOptions& options,
                        const TransformStamped& transform,
                        point_3d* translation, double* qx, double* qy,
                        double* qz, double* qw, std::string* error) {
    if (options.map_frame().empty() || options.camera_frame().empty() ||
        transform.header().frame_id() != options.map_frame() ||
        transform.child_frame_id() != options.camera_frame()) {
        set_error(error, "camera-to-map transform frames are invalid.");
        return false;
    }
    const auto& source_translation = transform.transform().translation();
    const auto& rotation = transform.transform().rotation();
    translation->x = source_translation.x();
    translation->y = source_translation.y();
    translation->z = source_translation.z();
    *qx = rotation.x();
    *qy = rotation.y();
    *qz = rotation.z();
    *qw = rotation.w();
    if (!std::isfinite(translation->x) || !std::isfinite(translation->y) ||
        !std::isfinite(translation->z) || !std::isfinite(*qx) ||
        !std::isfinite(*qy) || !std::isfinite(*qz) || !std::isfinite(*qw)) {
        set_error(error, "camera-to-map transform is non-finite.");
        return false;
    }
    const double norm =
        std::sqrt(*qx * *qx + *qy * *qy + *qz * *qz + *qw * *qw);
    if (!finite_positive(norm)) {
        set_error(error, "camera-to-map rotation is invalid.");
        return false;
    }
    *qx /= norm;
    *qy /= norm;
    *qz /= norm;
    *qw /= norm;
    return true;
}

point_3d transform_point(const point_3d& point, const point_3d& translation,
                         double qx, double qy, double qz, double qw) {
    const double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double r01 = 2.0 * (qx * qy - qz * qw);
    const double r02 = 2.0 * (qx * qz + qy * qw);
    const double r10 = 2.0 * (qx * qy + qz * qw);
    const double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
    const double r12 = 2.0 * (qy * qz - qx * qw);
    const double r20 = 2.0 * (qx * qz - qy * qw);
    const double r21 = 2.0 * (qy * qz + qx * qw);
    const double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
    point_3d transformed;
    transformed.x =
        translation.x + r00 * point.x + r01 * point.y + r02 * point.z;
    transformed.y =
        translation.y + r10 * point.x + r11 * point.y + r12 * point.z;
    transformed.z =
        translation.z + r20 * point.x + r21 * point.y + r22 * point.z;
    return transformed;
}

}  // namespace

TargetLocalizer::TargetLocalizer(const proto::ShadowOptions& options)
    : options_(options) {}

bool TargetLocalizer::EstimateRange(const Detection2D& detection,
                                    const Image& depth,
                                    const CameraInfo& camera, float* range_m,
                                    std::string* error) const {
    if (error != nullptr) {
        error->clear();
    }
    if (range_m == nullptr) {
        set_error(error, "range output is null.");
        return false;
    }
    *range_m = 0.0F;
    if (!validate_depth_options(options_, error)) {
        return false;
    }
    uint32_t bytes_per_pixel = 0;
    if (!validate_depth_image(depth, &bytes_per_pixel, error) ||
        !validate_camera(camera, depth, error)) {
        return false;
    }
    if ((!detection.header().frame_id().empty() &&
         detection.header().frame_id() != options_.camera_frame()) ||
        (!depth.header().frame_id().empty() &&
         depth.header().frame_id() != options_.camera_frame()) ||
        (!camera.header().frame_id().empty() &&
         camera.header().frame_id() != options_.camera_frame())) {
        set_error(
            error,
            "detection, depth, and camera frames must match camera_frame.");
        return false;
    }

    image_region region;
    if (!inner_region(detection, depth, options_.inner_box_scale(), &region,
                      error)) {
        return false;
    }
    std::vector<float> samples;
    samples.reserve(static_cast<size_t>(region.end_x - region.begin_x) *
                    (region.end_y - region.begin_y));
    for (uint32_t row = region.begin_y; row < region.end_y; ++row) {
        const char* row_data =
            depth.data().data() + static_cast<size_t>(row) * depth.step();
        for (uint32_t col = region.begin_x; col < region.end_x; ++col) {
            float value_m = 0.0F;
            const char* sample_data =
                row_data + static_cast<size_t>(col) * bytes_per_pixel;
            if (depth.encoding() == "16UC1") {
                const uint16_t raw_value =
                    decode_uint16(sample_data, depth.is_bigendian());
                value_m =
                    static_cast<float>(raw_value) * options_.depth_scale();
            } else {
                value_m = decode_float32(sample_data, depth.is_bigendian());
            }
            if (std::isfinite(value_m) && value_m >= options_.min_depth_m() &&
                value_m <= options_.max_depth_m()) {
                samples.push_back(value_m);
            }
        }
    }
    if (samples.size() < options_.min_depth_samples()) {
        set_error(error, "too few valid depth samples.");
        return false;
    }

    const float first_median = median(&samples);
    std::vector<float> inliers;
    inliers.reserve(samples.size());
    for (const float sample : samples) {
        if (std::abs(sample - first_median) <= options_.depth_outlier_m()) {
            inliers.push_back(sample);
        }
    }
    if (inliers.size() < options_.min_depth_samples()) {
        set_error(error,
                  "too few depth samples remain after outlier rejection.");
        return false;
    }
    *range_m = median(&inliers);
    return true;
}

bool TargetLocalizer::Locate(
    const std::string& target_id, int64_t stamp_ns,
    const Detection2D& detection, const Image& depth, const CameraInfo& camera,
    const TransformStamped& camera_to_map,
    automsgs::msgs::geometry_msgs::PoseStamped* pose,
    automsgs::msgs::geometry_msgs::TwistStamped* velocity, std::string* error) {
    if (error != nullptr) {
        error->clear();
    }
    if (pose != nullptr) {
        pose->Clear();
    }
    if (velocity != nullptr) {
        velocity->Clear();
    }
    if (pose == nullptr || velocity == nullptr) {
        set_error(error, "pose and velocity outputs must not be null.");
        return false;
    }
    if (has_history_ && target_id != target_id_) {
        reset_history();
    }
    if (target_id.empty()) {
        set_error(error, "target_id must not be empty.");
        return false;
    }
    if (has_history_ && stamp_ns <= stamp_ns_) {
        reset_history();
        set_error(error, "timestamp must increase for the selected target.");
        return false;
    }

    int32_t seconds = 0;
    uint32_t nanoseconds = 0;
    if (!split_stamp(stamp_ns, &seconds, &nanoseconds, error)) {
        return false;
    }
    float depth_m = 0.0F;
    if (!EstimateRange(detection, depth, camera, &depth_m, error)) {
        return false;
    }

    const double center_x = detection.bbox().center().position().x();
    const double center_y = detection.bbox().center().position().y();
    const point_3d camera_point{
        (center_x - camera.k(2)) * depth_m / camera.k(0),
        (center_y - camera.k(5)) * depth_m / camera.k(4), depth_m};
    point_3d translation;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 0.0;
    if (!validate_transform(options_, camera_to_map, &translation, &qx, &qy,
                            &qz, &qw, error)) {
        return false;
    }
    const point_3d measured =
        transform_point(camera_point, translation, qx, qy, qz, qw);
    if (!std::isfinite(measured.x) || !std::isfinite(measured.y) ||
        !std::isfinite(measured.z)) {
        set_error(error, "transformed target position is non-finite.");
        return false;
    }

    double filtered_x = measured.x;
    double filtered_y = measured.y;
    double filtered_z = measured.z;
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    if (has_history_) {
        const double dt_sec =
            static_cast<double>(static_cast<long double>(stamp_ns) -
                                static_cast<long double>(stamp_ns_)) /
            kNanosecondsPerSecond;
        if (!finite_positive(dt_sec)) {
            reset_history();
            set_error(error, "target localization interval is invalid.");
            return false;
        }
        filtered_x =
            position_x_ + kPositionFilterAlpha * (measured.x - position_x_);
        filtered_y =
            position_y_ + kPositionFilterAlpha * (measured.y - position_y_);
        filtered_z =
            position_z_ + kPositionFilterAlpha * (measured.z - position_z_);
        velocity_x = (filtered_x - position_x_) / dt_sec;
        velocity_y = (filtered_y - position_y_) / dt_sec;
    }
    if (!std::isfinite(filtered_x) || !std::isfinite(filtered_y) ||
        !std::isfinite(filtered_z) || !std::isfinite(velocity_x) ||
        !std::isfinite(velocity_y)) {
        reset_history();
        set_error(error, "filtered target state is non-finite.");
        return false;
    }

    automsgs::msgs::geometry_msgs::PoseStamped next_pose;
    next_pose.mutable_header()->set_frame_id(options_.map_frame());
    next_pose.mutable_header()->mutable_stamp()->set_sec(seconds);
    next_pose.mutable_header()->mutable_stamp()->set_nanosec(nanoseconds);
    next_pose.mutable_pose()->mutable_position()->set_x(filtered_x);
    next_pose.mutable_pose()->mutable_position()->set_y(filtered_y);
    next_pose.mutable_pose()->mutable_position()->set_z(filtered_z);
    next_pose.mutable_pose()->mutable_orientation()->set_x(0.0);
    next_pose.mutable_pose()->mutable_orientation()->set_y(0.0);
    next_pose.mutable_pose()->mutable_orientation()->set_z(0.0);
    next_pose.mutable_pose()->mutable_orientation()->set_w(1.0);

    automsgs::msgs::geometry_msgs::TwistStamped next_velocity;
    next_velocity.mutable_header()->set_frame_id(options_.map_frame());
    next_velocity.mutable_header()->mutable_stamp()->set_sec(seconds);
    next_velocity.mutable_header()->mutable_stamp()->set_nanosec(nanoseconds);
    next_velocity.mutable_twist()->mutable_linear()->set_x(velocity_x);
    next_velocity.mutable_twist()->mutable_linear()->set_y(velocity_y);
    next_velocity.mutable_twist()->mutable_linear()->set_z(0.0);
    next_velocity.mutable_twist()->mutable_angular()->set_x(0.0);
    next_velocity.mutable_twist()->mutable_angular()->set_y(0.0);
    next_velocity.mutable_twist()->mutable_angular()->set_z(0.0);

    target_id_ = target_id;
    stamp_ns_ = stamp_ns;
    position_x_ = filtered_x;
    position_y_ = filtered_y;
    position_z_ = filtered_z;
    has_history_ = true;
    *pose = std::move(next_pose);
    *velocity = std::move(next_velocity);
    return true;
}

void TargetLocalizer::Clear() {
    reset_history();
}

void TargetLocalizer::reset_history() {
    has_history_ = false;
    target_id_.clear();
    stamp_ns_ = 0;
    position_x_ = 0.0;
    position_y_ = 0.0;
    position_z_ = 0.0;
}

}  // namespace shadow
}  // namespace perception
}  // namespace autonomy
