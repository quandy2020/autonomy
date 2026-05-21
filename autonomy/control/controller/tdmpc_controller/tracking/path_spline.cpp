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

#include "autonomy/control/controller/tdmpc_controller/tracking/path_spline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "autonomy/common/math/math.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {
namespace tracking {

bool PathSpline::BuildFromPath(const commsgs::planning_msgs::Path& path) {
    path_ = path;
    cumulative_.clear();
    if (path_.poses.empty()) {
        return false;
    }
    cumulative_.resize(path_.poses.size(), 0.0);
    for (size_t i = 1; i < path_.poses.size(); ++i) {
        cumulative_[i] =
            cumulative_[i - 1] +
            map::costmap_2d::utils::euclidean_distance(path_.poses[i - 1],
                                                       path_.poses[i]);
    }
    return true;
}

double PathSpline::length() const {
    return cumulative_.empty() ? 0.0 : cumulative_.back();
}

size_t PathSpline::FindClosestIndex(const Pose2D& query) const {
    size_t best = 0;
    double best_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_.poses.size(); ++i) {
        const double dx =
            static_cast<double>(path_.poses[i].pose.position.x) - query.x;
        const double dy =
            static_cast<double>(path_.poses[i].pose.position.y) - query.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_dist_sq) {
            best_dist_sq = d2;
            best = i;
        }
    }
    return best;
}

double PathSpline::ArcLengthAtIndex(size_t index) const {
    if (cumulative_.empty()) {
        return 0.0;
    }
    index = std::min(index, cumulative_.size() - 1);
    return cumulative_[index];
}

Pose2D PathSpline::PoseAtArcLength(double s) const {
    Pose2D pose;
    if (path_.poses.empty()) {
        return pose;
    }
    const double total = length();
    if (s <= 0.0) {
        const auto& p = path_.poses.front();
        pose.x = static_cast<double>(p.pose.position.x);
        pose.y = static_cast<double>(p.pose.position.y);
        pose.theta = transform::tf2::getYaw(p.pose.orientation);
        return pose;
    }
    if (s >= total) {
        const auto& p = path_.poses.back();
        pose.x = static_cast<double>(p.pose.position.x);
        pose.y = static_cast<double>(p.pose.position.y);
        pose.theta = transform::tf2::getYaw(p.pose.orientation);
        return pose;
    }

    size_t segment_idx = 0;
    while (segment_idx + 1 < cumulative_.size() &&
           cumulative_[segment_idx + 1] < s) {
        ++segment_idx;
    }
    const size_t next_idx = std::min(segment_idx + 1, path_.poses.size() - 1);
    const double seg_start = cumulative_[segment_idx];
    const double seg_len =
        std::max(cumulative_[next_idx] - seg_start, 1e-6);
    const double ratio = (s - seg_start) / seg_len;

    const auto& p0 = path_.poses[segment_idx].pose.position;
    const auto& p1 = path_.poses[next_idx].pose.position;
    pose.x = static_cast<double>(p0.x) +
             ratio * (static_cast<double>(p1.x) - static_cast<double>(p0.x));
    pose.y = static_cast<double>(p0.y) +
             ratio * (static_cast<double>(p1.y) - static_cast<double>(p0.y));

    const double yaw0 =
        transform::tf2::getYaw(path_.poses[segment_idx].pose.orientation);
    const double yaw1 =
        transform::tf2::getYaw(path_.poses[next_idx].pose.orientation);
    pose.theta = yaw0 + ratio * ::autonomy::common::NormalizeAngleDifference(yaw1 - yaw0);
    return pose;
}

void PathSpline::NormalAtArcLength(double s, double* nx, double* ny) const {
    const Pose2D ref = PoseAtArcLength(s);
    const Pose2D ahead = PoseAtArcLength(s + 0.05);
    const double tx = ahead.x - ref.x;
    const double ty = ahead.y - ref.y;
    const double norm = std::hypot(tx, ty);
    if (norm < 1e-9) {
        *nx = -std::sin(ref.theta);
        *ny = std::cos(ref.theta);
        return;
    }
    *nx = -ty / norm;
    *ny = tx / norm;
}

void PathSpline::ContouringErrorsAtArcLength(const Pose2D& state, double s,
                                             double* contour, double* lag) const {
    const Pose2D ref = PoseAtArcLength(s);
    double nx = 0.0;
    double ny = 0.0;
    NormalAtArcLength(s, &nx, &ny);
    const double dx = state.x - ref.x;
    const double dy = state.y - ref.y;
    *lag = dx * nx + dy * ny;
    const double tx = std::cos(ref.theta);
    const double ty = std::sin(ref.theta);
    *contour = dx * tx + dy * ty;
}

void PathSpline::ContouringErrors(const Pose2D& state, double* contour,
                                  double* lag) const {
    const size_t idx = FindClosestIndex(state);
    ContouringErrorsAtArcLength(state, ArcLengthAtIndex(idx), contour, lag);
}

}  // namespace tracking
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
