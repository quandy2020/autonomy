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

#include "autonomy/perception/follow/localizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace autonomy {
namespace perception {
namespace follow {
namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Follow: " + message;
    }
}

bool DepthAt(const automsgs::msgs::sensor_msgs::Image& depth, int x, int y,
             float* value) {
    if (depth.encoding() != "32FC1" || value == nullptr) {
        return false;
    }
    if (x < 0 || y < 0 || x >= static_cast<int>(depth.width()) ||
        y >= static_cast<int>(depth.height())) {
        return false;
    }
    const size_t index =
        (static_cast<size_t>(y) * depth.width() + static_cast<size_t>(x)) *
        sizeof(float);
    if (index + sizeof(float) > depth.data().size()) {
        return false;
    }
    float z = 0.0F;
    std::memcpy(&z, depth.data().data() + index, sizeof(float));
    *value = z;
    return std::isfinite(z);
}

}  // namespace

Localizer::Localizer(proto::FollowOptions options)
    : options_(std::move(options)) {}

bool Localizer::Localize(
    const automsgs::msgs::vision_msgs::Detection2D& track,
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& camera,
    const automsgs::msgs::geometry_msgs::TransformStamped& camera_to_map,
    automsgs::msgs::geometry_msgs::PoseStamped* target,
    std::string* error) const {
    if (target == nullptr) {
        SetError(error, "target must not be null.");
        return false;
    }
    if (camera.k_size() < 9) {
        SetError(error, "CameraInfo.K is incomplete.");
        return false;
    }
    const float cx = static_cast<float>(track.bbox().center().position().x());
    const float cy = static_cast<float>(track.bbox().center().position().y());
    const float sx = static_cast<float>(track.bbox().size_x());
    const float sy = static_cast<float>(track.bbox().size_y());
    if (!(sx > 1.0F) || !(sy > 1.0F)) {
        SetError(error, "track box is empty.");
        return false;
    }

    const float scale = std::clamp(options_.inner_box_scale(), 0.1F, 1.0F);
    const int x0 = static_cast<int>(std::floor(cx - 0.5F * scale * sx));
    const int x1 = static_cast<int>(std::ceil(cx + 0.5F * scale * sx));
    const int y0 = static_cast<int>(std::floor(cy - 0.5F * scale * sy));
    const int y1 = static_cast<int>(std::ceil(cy + 0.5F * scale * sy));

    std::vector<float> samples;
    samples.reserve(static_cast<size_t>((x1 - x0 + 1) * (y1 - y0 + 1)));
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            float z = 0.0F;
            if (!DepthAt(depth, x, y, &z)) {
                continue;
            }
            if (z < options_.min_depth_m() || z > options_.max_depth_m()) {
                continue;
            }
            samples.push_back(z);
        }
    }
    if (samples.size() < options_.min_depth_samples()) {
        SetError(error, "not enough depth samples in track box.");
        return false;
    }
    std::nth_element(samples.begin(),
                     samples.begin() + static_cast<std::ptrdiff_t>(samples.size() / 2),
                     samples.end());
    const float median = samples[samples.size() / 2];
    std::vector<float> filtered;
    filtered.reserve(samples.size());
    for (float z : samples) {
        if (std::fabs(z - median) <= options_.depth_outlier_m()) {
            filtered.push_back(z);
        }
    }
    if (filtered.size() < options_.min_depth_samples()) {
        SetError(error, "depth samples failed outlier rejection.");
        return false;
    }
    std::nth_element(filtered.begin(),
                     filtered.begin() +
                         static_cast<std::ptrdiff_t>(filtered.size() / 2),
                     filtered.end());
    const float z = filtered[filtered.size() / 2];

    const double fx = camera.k(0);
    const double fy = camera.k(4);
    const double ppx = camera.k(2);
    const double ppy = camera.k(5);
    if (!(fx > 1e-3) || !(fy > 1e-3)) {
        SetError(error, "invalid camera intrinsics.");
        return false;
    }
    const double X = (static_cast<double>(cx) - ppx) * z / fx;
    const double Y = (static_cast<double>(cy) - ppy) * z / fy;
    const double Z = z;

    const auto& t = camera_to_map.transform().translation();
    const auto& q = camera_to_map.transform().rotation();
    // Rotate then translate: p_map = R * p_cam + t
    const double qw = q.w();
    const double qx = q.x();
    const double qy = q.y();
    const double qz = q.z();
    const double rx =
        (1 - 2 * (qy * qy + qz * qz)) * X +
        (2 * (qx * qy - qz * qw)) * Y + (2 * (qx * qz + qy * qw)) * Z;
    const double ry =
        (2 * (qx * qy + qz * qw)) * X +
        (1 - 2 * (qx * qx + qz * qz)) * Y + (2 * (qy * qz - qx * qw)) * Z;
    const double rz =
        (2 * (qx * qz - qy * qw)) * X + (2 * (qy * qz + qx * qw)) * Y +
        (1 - 2 * (qx * qx + qy * qy)) * Z;

    target->Clear();
    target->mutable_header()->CopyFrom(depth.header());
    target->mutable_header()->set_frame_id(options_.map_frame());
    target->mutable_pose()->mutable_position()->set_x(rx + t.x());
    target->mutable_pose()->mutable_position()->set_y(ry + t.y());
    target->mutable_pose()->mutable_position()->set_z(rz + t.z());
    target->mutable_pose()->mutable_orientation()->set_w(1.0);
    return true;
}

}  // namespace follow
}  // namespace perception
}  // namespace autonomy
