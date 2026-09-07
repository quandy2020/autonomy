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
 * @file lift.cpp
 * @brief Median-depth AABB lift for Hestia detections.
 */

#include "autonomy/perception/hestia/lift.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace autonomy {
namespace perception {
namespace hestia {
namespace {

using CameraInfo = automsgs::msgs::sensor_msgs::CameraInfo;
using Detection2D = automsgs::msgs::vision_msgs::Detection2D;
using Detection3D = automsgs::msgs::vision_msgs::Detection3D;
using Image = automsgs::msgs::sensor_msgs::Image;
using TransformStamped = automsgs::msgs::geometry_msgs::TransformStamped;

struct ImageRegion {
    uint32_t begin_x = 0;
    uint32_t end_x = 0;
    uint32_t begin_y = 0;
    uint32_t end_y = 0;
};

struct Point3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Hestia lifter: " + message;
    }
}

bool FinitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

uint16_t DecodeUint16(const char* data, bool big_endian) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(data);
    if (big_endian) {
        return static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8) |
                                     static_cast<uint16_t>(bytes[1]));
    }
    return static_cast<uint16_t>(static_cast<uint16_t>(bytes[0]) |
                                 (static_cast<uint16_t>(bytes[1]) << 8));
}

float DecodeFloat32(const char* data, bool big_endian) {
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

bool ValidateDepthImage(const Image& depth, uint32_t* bytes_per_pixel,
                        std::string* error) {
    if (depth.encoding() == "16UC1") {
        *bytes_per_pixel = sizeof(uint16_t);
    } else if (depth.encoding() == "32FC1") {
        *bytes_per_pixel = sizeof(float);
    } else {
        SetError(error, "depth encoding must be '16UC1' or '32FC1'.");
        return false;
    }
    if (depth.width() == 0 || depth.height() == 0) {
        SetError(error, "depth dimensions must be positive.");
        return false;
    }
    const size_t minimum_step =
        static_cast<size_t>(depth.width()) * *bytes_per_pixel;
    if (depth.step() < minimum_step) {
        SetError(error, "depth image step is too small.");
        return false;
    }
    const size_t required_size =
        static_cast<size_t>(depth.height()) * depth.step();
    if (depth.data().size() < required_size) {
        SetError(error, "depth image data is too small.");
        return false;
    }
    return true;
}

bool ValidateCamera(const CameraInfo& camera, const Image& depth,
                    std::string* error) {
    if (camera.width() == 0 || camera.height() == 0 ||
        camera.width() != depth.width() || camera.height() != depth.height()) {
        SetError(error, "camera dimensions must be positive and match depth.");
        return false;
    }
    if (camera.k_size() < 9 || !FinitePositive(camera.k(0)) ||
        !FinitePositive(camera.k(4)) || !std::isfinite(camera.k(2)) ||
        !std::isfinite(camera.k(5))) {
        SetError(error, "camera intrinsics are invalid.");
        return false;
    }
    return true;
}

bool InnerRegion(const Detection2D& detection, const Image& depth, float scale,
                 ImageRegion* region, std::string* error) {
    const auto& box = detection.bbox();
    const double center_x = box.center().position().x();
    const double center_y = box.center().position().y();
    const double width = box.size_x();
    const double height = box.size_y();
    if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
        !FinitePositive(width) || !FinitePositive(height)) {
        SetError(error, "detection bounding box is invalid.");
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
        SetError(error, "detection inner box contains no depth pixels.");
        return false;
    }
    return true;
}

float Percentile(std::vector<float> values, double fraction) {
    if (values.empty()) {
        return 0.0F;
    }
    std::sort(values.begin(), values.end());
    const double index =
        std::clamp(fraction, 0.0, 1.0) * static_cast<double>(values.size() - 1);
    const size_t lower = static_cast<size_t>(std::floor(index));
    const size_t upper = static_cast<size_t>(std::ceil(index));
    if (lower == upper) {
        return values[lower];
    }
    const double weight = index - static_cast<double>(lower);
    return static_cast<float>((1.0 - weight) * values[lower] +
                              weight * values[upper]);
}

float Median(std::vector<float>* values) {
    return Percentile(*values, 0.5);
}

Point3d TransformPoint(const Point3d& point, const Point3d& translation,
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
    Point3d transformed;
    transformed.x =
        translation.x + r00 * point.x + r01 * point.y + r02 * point.z;
    transformed.y =
        translation.y + r10 * point.x + r11 * point.y + r12 * point.z;
    transformed.z =
        translation.z + r20 * point.x + r21 * point.y + r22 * point.z;
    return transformed;
}

bool TryParseTransform(const TransformStamped& transform,
                       const std::string& camera_frame,
                       const std::string& base_frame, Point3d* translation,
                       double* qx, double* qy, double* qz, double* qw) {
    if (base_frame.empty() || transform.header().frame_id() != base_frame ||
        transform.child_frame_id() != camera_frame) {
        return false;
    }
    translation->x = transform.transform().translation().x();
    translation->y = transform.transform().translation().y();
    translation->z = transform.transform().translation().z();
    *qx = transform.transform().rotation().x();
    *qy = transform.transform().rotation().y();
    *qz = transform.transform().rotation().z();
    *qw = transform.transform().rotation().w();
    if (!std::isfinite(translation->x) || !std::isfinite(translation->y) ||
        !std::isfinite(translation->z) || !std::isfinite(*qx) ||
        !std::isfinite(*qy) || !std::isfinite(*qz) || !std::isfinite(*qw)) {
        return false;
    }
    const double norm =
        std::sqrt(*qx * *qx + *qy * *qy + *qz * *qz + *qw * *qw);
    if (!FinitePositive(norm)) {
        return false;
    }
    *qx /= norm;
    *qy /= norm;
    *qz /= norm;
    *qw /= norm;
    return true;
}

bool CollectSamples(const Image& depth, uint32_t bytes_per_pixel,
                    const ImageRegion& region, float depth_scale,
                    float min_depth_m, float max_depth_m,
                    std::vector<float>* samples) {
    samples->clear();
    for (uint32_t y = region.begin_y; y < region.end_y; ++y) {
        for (uint32_t x = region.begin_x; x < region.end_x; ++x) {
            const size_t offset = static_cast<size_t>(y) * depth.step() +
                                  static_cast<size_t>(x) * bytes_per_pixel;
            float depth_m = 0.0F;
            if (bytes_per_pixel == sizeof(uint16_t)) {
                const uint16_t raw =
                    DecodeUint16(depth.data().data() + offset,
                                 depth.is_bigendian());
                depth_m = static_cast<float>(raw) * depth_scale;
            } else {
                depth_m = DecodeFloat32(depth.data().data() + offset,
                                        depth.is_bigendian());
            }
            if (!std::isfinite(depth_m) || depth_m < min_depth_m ||
                depth_m > max_depth_m) {
                continue;
            }
            samples->push_back(depth_m);
        }
    }
    return !samples->empty();
}

bool RejectOutliers(std::vector<float>* samples, float outlier_m,
                    float* median_depth) {
    if (samples->empty()) {
        return false;
    }
    *median_depth = Median(samples);
    std::vector<float> inliers;
    inliers.reserve(samples->size());
    for (float sample : *samples) {
        if (std::fabs(sample - *median_depth) <= outlier_m) {
            inliers.push_back(sample);
        }
    }
    if (inliers.empty()) {
        return false;
    }
    *samples = std::move(inliers);
    *median_depth = Median(samples);
    return true;
}

bool LiftOne(const Detection2D& detection, const Image& depth,
             const CameraInfo& camera, uint32_t bytes_per_pixel,
             const proto::HestiaOptions& options, bool use_base,
             const Point3d& translation, double qx, double qy, double qz,
             double qw, Detection3D* out) {
    ImageRegion region;
    std::string ignored;
    if (!InnerRegion(detection, depth, options.inner_box_scale(), &region,
                     &ignored)) {
        return false;
    }
    std::vector<float> samples;
    if (!CollectSamples(depth, bytes_per_pixel, region, options.depth_scale(),
                        options.min_depth_m(), options.max_depth_m(),
                        &samples)) {
        return false;
    }
    float median_depth = 0.0F;
    if (!RejectOutliers(&samples, options.depth_outlier_m(), &median_depth)) {
        return false;
    }
    if (samples.size() < options.min_depth_samples()) {
        return false;
    }

    const double fx = camera.k(0);
    const double fy = camera.k(4);
    const double cx = camera.k(2);
    const double cy = camera.k(5);
    const double u = detection.bbox().center().position().x();
    const double v = detection.bbox().center().position().y();
    Point3d center_cam;
    center_cam.z = median_depth;
    center_cam.x = (u - cx) * center_cam.z / fx;
    center_cam.y = (v - cy) * center_cam.z / fy;

    const float depth_span =
        std::max(0.01F, Percentile(samples, 0.9) - Percentile(samples, 0.1));
    const double size_x =
        detection.bbox().size_x() * center_cam.z / fx;
    const double size_y =
        detection.bbox().size_y() * center_cam.z / fy;
    const double size_z = depth_span;

    Point3d center = center_cam;
    std::string frame_id = options.camera_frame();
    if (use_base) {
        center = TransformPoint(center_cam, translation, qx, qy, qz, qw);
        frame_id = options.base_frame();
    }

    out->Clear();
    *out->mutable_header() = detection.header();
    out->mutable_header()->set_frame_id(frame_id);
    out->set_id(detection.id());
    for (const auto& result : detection.results()) {
        *out->add_results() = result;
    }
    auto* bbox = out->mutable_bbox();
    bbox->mutable_center()->mutable_position()->set_x(center.x);
    bbox->mutable_center()->mutable_position()->set_y(center.y);
    bbox->mutable_center()->mutable_position()->set_z(center.z);
    bbox->mutable_center()->mutable_orientation()->set_w(1.0);
    bbox->mutable_size()->set_x(size_x);
    bbox->mutable_size()->set_y(size_y);
    bbox->mutable_size()->set_z(size_z);
    return true;
}

}  // namespace

Lifter::Lifter(const proto::HestiaOptions& options)
    : options_(options) {}

bool Lifter::Lift(
    const automsgs::msgs::vision_msgs::Detection2DArray& detections_2d,
    const Image& depth, const CameraInfo& camera,
    const TransformStamped* camera_to_base,
    automsgs::msgs::vision_msgs::Detection3DArray* detections_3d,
    std::string* error) const {
    if (error != nullptr) {
        error->clear();
    }
    if (detections_3d == nullptr) {
        SetError(error, "detection3d output is null.");
        return false;
    }
    detections_3d->Clear();
    *detections_3d->mutable_header() = detections_2d.header();
    detections_3d->mutable_header()->set_frame_id(options_.camera_frame());

    uint32_t bytes_per_pixel = 0;
    if (!ValidateDepthImage(depth, &bytes_per_pixel, error) ||
        !ValidateCamera(camera, depth, error)) {
        return false;
    }

    Point3d translation;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    const bool use_base =
        camera_to_base != nullptr &&
        TryParseTransform(*camera_to_base, options_.camera_frame(),
                          options_.base_frame(), &translation, &qx, &qy, &qz,
                          &qw);
    if (use_base) {
        detections_3d->mutable_header()->set_frame_id(options_.base_frame());
    }

    for (const auto& detection : detections_2d.detections()) {
        Detection3D lifted;
        if (!LiftOne(detection, depth, camera, bytes_per_pixel, options_,
                     use_base, translation, qx, qy, qz, qw, &lifted)) {
            continue;
        }
        *detections_3d->add_detections() = std::move(lifted);
    }
    return true;
}

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy
