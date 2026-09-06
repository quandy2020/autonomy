/*
 * Copyright 2026 Autodriver contributors
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

#include "autodriver/lidar/motion_compensator.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace autodriver {
namespace lidar {
namespace {

struct FieldLayout {
    int x = -1;
    int y = -1;
    int z = -1;
    int intensity = -1;
    int timestamp = -1;
    std::uint32_t point_step = 0;
};

int FindFieldOffset(const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
                    const std::string& name) {
    for (const auto& field : cloud.fields()) {
        if (field.name() == name) {
            return static_cast<int>(field.offset());
        }
    }
    return -1;
}

FieldLayout LayoutOf(const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
    FieldLayout layout;
    layout.x = FindFieldOffset(cloud, "x");
    layout.y = FindFieldOffset(cloud, "y");
    layout.z = FindFieldOffset(cloud, "z");
    layout.intensity = FindFieldOffset(cloud, "intensity");
    layout.timestamp = FindFieldOffset(cloud, "timestamp");
    layout.point_step = cloud.point_step();
    return layout;
}

float ReadF32(const char* p) {
    float v = 0;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

double ReadF64(const char* p) {
    double v = 0;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

void WriteF32(char* p, float v) { std::memcpy(p, &v, sizeof(v)); }

void WriteF64(char* p, double v) { std::memcpy(p, &v, sizeof(v)); }

}  // namespace

MotionCompensator::MotionCompensator(CompensatorOptions options)
    : options_(std::move(options)) {}

void MotionCompensator::SetPoseLookup(PoseLookup lookup) {
    lookup_ = std::move(lookup);
}

bool MotionCompensator::Compensate(
    const automsgs::msgs::sensor_msgs::PointCloud2& in,
    automsgs::msgs::sensor_msgs::PointCloud2* out) const {
    if (out == nullptr || !lookup_ || in.width() == 0 || in.point_step() == 0) {
        return false;
    }
    const FieldLayout layout = LayoutOf(in);
    if (layout.x < 0 || layout.y < 0 || layout.z < 0 || layout.timestamp < 0) {
        return false;
    }

    const std::size_t n = static_cast<std::size_t>(in.width()) *
                          std::max<std::uint32_t>(in.height(), 1);
    const char* data = in.data().data();
    std::uint64_t t_min = std::numeric_limits<std::uint64_t>::max();
    std::uint64_t t_max = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const char* pt = data + i * layout.point_step;
        const auto tp =
            static_cast<std::uint64_t>(ReadF64(pt + layout.timestamp));
        t_min = std::min(t_min, tp);
        t_max = std::max(t_max, tp);
    }
    if (t_max <= t_min) {
        *out = in;
        return true;
    }

    const std::string child =
        in.header().frame_id().empty() ? "lidar" : in.header().frame_id();
    Eigen::Affine3d pose_min = Eigen::Affine3d::Identity();
    Eigen::Affine3d pose_max = Eigen::Affine3d::Identity();
    if (!lookup_(t_min, child, &pose_min) ||
        !lookup_(t_max, child, &pose_max)) {
        return false;
    }

    Eigen::Vector3d translation =
        pose_min.translation() - pose_max.translation();
    Eigen::Quaterniond q_max(pose_max.linear());
    Eigen::Quaterniond q_min(pose_min.linear());
    Eigen::Quaterniond q1(q_max.conjugate() * q_min);
    q1.normalize();
    translation = q_max.conjugate() * translation;
    const Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    const double d = q0.dot(q1);
    const double abs_d = std::abs(d);
    const double f = 1.0 / static_cast<double>(t_max - t_min);

    *out = in;
    char* out_data = out->mutable_data()->data();
    const bool significant_rotation = abs_d < 1.0 - 1.0e-8;
    const double theta = significant_rotation ? std::acos(abs_d) : 0.0;
    const double sin_theta =
        significant_rotation ? std::sin(theta) : 1.0;
    const double c1_sign = d > 0 ? 1.0 : -1.0;

    for (std::size_t i = 0; i < n; ++i) {
        char* pt = out_data + i * layout.point_step;
        const float x = ReadF32(pt + layout.x);
        if (std::isnan(x)) {
            continue;
        }
        const float y = ReadF32(pt + layout.y);
        const float z = ReadF32(pt + layout.z);
        Eigen::Vector3d p(x, y, z);
        const auto tp =
            static_cast<std::uint64_t>(ReadF64(pt + layout.timestamp));
        const double t = static_cast<double>(t_max - tp) * f;

        if (significant_rotation) {
            const double c0 = std::sin((1.0 - t) * theta) / sin_theta;
            const double c1 = std::sin(t * theta) / sin_theta * c1_sign;
            Eigen::Quaterniond qi(c0 * q0.coeffs() + c1 * q1.coeffs());
            Eigen::Affine3d trans = Eigen::Translation3d(t * translation) * qi;
            p = trans * p;
        } else {
            p = Eigen::Translation3d(t * translation) * p;
        }
        WriteF32(pt + layout.x, static_cast<float>(p.x()));
        WriteF32(pt + layout.y, static_cast<float>(p.y()));
        WriteF32(pt + layout.z, static_cast<float>(p.z()));
    }
    return true;
}

PoseLookup MakeLinearPoseLookup(std::uint64_t t_min, std::uint64_t t_max,
                                const Eigen::Affine3d& pose_min,
                                const Eigen::Affine3d& pose_max) {
    return [t_min, t_max, pose_min, pose_max](std::uint64_t time_ns,
                                              const std::string&,
                                              Eigen::Affine3d* pose) {
        if (pose == nullptr || t_max <= t_min) {
            return false;
        }
        const double alpha = std::clamp(
            static_cast<double>(time_ns - t_min) /
                static_cast<double>(t_max - t_min),
            0.0, 1.0);
        Eigen::Quaterniond q_min(pose_min.linear());
        Eigen::Quaterniond q_max(pose_max.linear());
        Eigen::Quaterniond q = q_min.slerp(alpha, q_max);
        Eigen::Vector3d p =
            (1.0 - alpha) * pose_min.translation() + alpha * pose_max.translation();
        *pose = Eigen::Translation3d(p) * q;
        return true;
    };
}

}  // namespace lidar
}  // namespace autodriver
