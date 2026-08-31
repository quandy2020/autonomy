/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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
 * @file path_reference.cpp
 * @brief Implementation of nmpc_controller::PathReference
 */

#include "autonomy/control/controller/nmpc_controller/path_reference.hpp"

#include <algorithm>
#include <limits>

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc_controller {

double PathReference::YawFromPose(
    const automsgs::msgs::geometry_msgs::Pose& pose) {
    const auto& q = pose.orientation();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

void PathReference::SetPlan(const automsgs::msgs::nav_msgs::Path& plan) {
    poses_.clear();
    cumulative_s_.clear();
    path_length_ = 0.0;
    goal_pose_.Clear();

    poses_.reserve(plan.poses_size());
    cumulative_s_.reserve(plan.poses_size());
    for (const auto& stamped : plan.poses()) {
        Pose2D p;
        p.x = stamped.pose().position().x();
        p.y = stamped.pose().position().y();
        p.yaw = YawFromPose(stamped.pose());
        if (!poses_.empty()) {
            const auto& prev = poses_.back();
            path_length_ += std::hypot(p.x - prev.x, p.y - prev.y);
        }
        poses_.push_back(p);
        cumulative_s_.push_back(path_length_);
    }
    if (!plan.poses().empty()) {
        goal_pose_ = plan.poses(plan.poses_size() - 1).pose();
    }
}

PathReference::ClosestPoint PathReference::ClosestPointOnPath(
    double x, double y, double min_arc_length, double search_behind) const {
    ClosestPoint result;
    if (poses_.empty()) {
        return result;
    }
    if (poses_.size() == 1) {
        result.arc_length = 0.0;
        result.distance =
            std::hypot(x - poses_.front().x, y - poses_.front().y);
        return result;
    }

    const double search_start =
        std::max(0.0, min_arc_length - std::max(search_behind, 0.0));
    size_t start_idx = 0;
    while (start_idx + 1 < poses_.size() &&
           cumulative_s_[start_idx + 1] < search_start) {
        ++start_idx;
    }

    double best_dist_sq = std::numeric_limits<double>::infinity();
    for (size_t i = std::max<size_t>(1, start_idx); i < poses_.size(); ++i) {
        const auto& p0 = poses_[i - 1];
        const auto& p1 = poses_[i];
        const double dx = p1.x - p0.x;
        const double dy = p1.y - p0.y;
        const double seg_len_sq = dx * dx + dy * dy;
        double t = 0.0;
        if (seg_len_sq > 1e-12) {
            t = ((x - p0.x) * dx + (y - p0.y) * dy) / seg_len_sq;
            t = std::clamp(t, 0.0, 1.0);
        }
        const double proj_x = p0.x + t * dx;
        const double proj_y = p0.y + t * dy;
        const double dist_sq =
            (x - proj_x) * (x - proj_x) + (y - proj_y) * (y - proj_y);
        const double arc_length =
            cumulative_s_[i - 1] + t * std::sqrt(seg_len_sq);
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            result.arc_length = arc_length;
            result.distance = std::sqrt(dist_sq);
        }
    }
    return result;
}

PathReference::Pose2D PathReference::SampleAlongPath(double start_s, double step,
                                                     int count) const {
    if (poses_.empty()) {
        return {};
    }
    const double target_s = start_s + step * static_cast<double>(count);
    if (target_s >= path_length_) {
        return poses_.back();
    }
    auto it = std::lower_bound(cumulative_s_.begin(), cumulative_s_.end(), target_s);
    const size_t idx = static_cast<size_t>(std::distance(cumulative_s_.begin(), it));
    if (idx == 0) {
        return poses_.front();
    }
    const double s0 = cumulative_s_[idx - 1];
    const double s1 = cumulative_s_[idx];
    const double ratio = (s1 > s0) ? (target_s - s0) / (s1 - s0) : 0.0;
    Pose2D out;
    out.x = poses_[idx - 1].x + ratio * (poses_[idx].x - poses_[idx - 1].x);
    out.y = poses_[idx - 1].y + ratio * (poses_[idx].y - poses_[idx - 1].y);
    double dyaw = poses_[idx].yaw - poses_[idx - 1].yaw;
    while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
    while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
    out.yaw = poses_[idx - 1].yaw + ratio * dyaw;
    return out;
}

PathReference::Pose2D PathReference::Goal() const {
    return poses_.empty() ? Pose2D{} : poses_.back();
}

}  // namespace nmpc_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
