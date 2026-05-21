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

#include "autonomy/control/controller/nmpc_controller/tracking/path_reference.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/common/math/math.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace tracking {

PathReference::PathReference(int horizon, double dt, double reference_velocity,
                             double slowdown_radius)
    : horizon_(horizon > 0 ? horizon : 1),
      dt_(dt > 0.0 ? dt : 0.1),
      reference_velocity_(reference_velocity > 0.0 ? reference_velocity : 0.1),
      slowdown_radius_(slowdown_radius > 0.0 ? slowdown_radius : 0.0),
      effective_reference_velocity_(reference_velocity_) {}

Pose2D PathReference::InterpolateAlongPath(
    const commsgs::planning_msgs::Path& path,
    const std::vector<double>& cumulative, double arc_length) {
    Pose2D ref;
    if (path.poses.empty()) {
        return ref;
    }

    const double total_length = cumulative.back();
    if (arc_length <= 0.0) {
        const auto& p = path.poses.front();
        ref.x = static_cast<double>(p.pose.position.x);
        ref.y = static_cast<double>(p.pose.position.y);
        ref.theta = transform::tf2::getYaw(p.pose.orientation);
        return ref;
    }
    if (arc_length >= total_length) {
        const auto& p = path.poses.back();
        ref.x = static_cast<double>(p.pose.position.x);
        ref.y = static_cast<double>(p.pose.position.y);
        ref.theta = transform::tf2::getYaw(p.pose.orientation);
        return ref;
    }

    size_t segment_idx = 0;
    while (segment_idx + 1 < cumulative.size() &&
           cumulative[segment_idx + 1] < arc_length) {
        ++segment_idx;
    }

    const size_t next_idx = std::min(segment_idx + 1, path.poses.size() - 1);
    const double seg_start = cumulative[segment_idx];
    const double seg_len =
        std::max(cumulative[next_idx] - seg_start, 1e-6);
    const double ratio = (arc_length - seg_start) / seg_len;

    const auto& p0 = path.poses[segment_idx].pose.position;
    const auto& p1 = path.poses[next_idx].pose.position;
    ref.x = static_cast<double>(p0.x) +
            ratio * (static_cast<double>(p1.x) - static_cast<double>(p0.x));
    ref.y = static_cast<double>(p0.y) +
            ratio * (static_cast<double>(p1.y) - static_cast<double>(p0.y));

    const double yaw0 =
        transform::tf2::getYaw(path.poses[segment_idx].pose.orientation);
    const double yaw1 =
        transform::tf2::getYaw(path.poses[next_idx].pose.orientation);
    ref.theta = yaw0 + ratio * ::autonomy::common::NormalizeAngleDifference(yaw1 - yaw0);
    return ref;
}

bool PathReference::BuildFromPath(const commsgs::planning_msgs::Path& path,
                                  const Pose2D& current_state) {
    references_.clear();
    references_.resize(static_cast<size_t>(horizon_ + 1));

    if (path.poses.empty()) {
        references_[0] = current_state;
        for (int k = 1; k <= horizon_; ++k) {
            references_[static_cast<size_t>(k)] = references_[0];
        }
        effective_reference_velocity_ = 0.0;
        return false;
    }

    const auto& poses = path.poses;
    std::vector<double> cumulative(poses.size(), 0.0);
    for (size_t i = 1; i < poses.size(); ++i) {
        cumulative[i] =
            cumulative[i - 1] +
            map::costmap_2d::utils::euclidean_distance(poses[i - 1], poses[i]);
    }

    const double total_length = cumulative.back();

    size_t closest_idx = 0;
    double closest_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < poses.size(); ++i) {
        const double dx =
            static_cast<double>(poses[i].pose.position.x) - current_state.x;
        const double dy =
            static_cast<double>(poses[i].pose.position.y) - current_state.y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < closest_dist_sq) {
            closest_dist_sq = dist_sq;
            closest_idx = i;
        }
    }

    const double arc_origin = cumulative[closest_idx];
    const double dist_to_goal = std::max(total_length - arc_origin, 0.0);
    effective_reference_velocity_ = reference_velocity_;
    if (slowdown_radius_ > 0.0 && dist_to_goal < slowdown_radius_) {
        effective_reference_velocity_ =
            reference_velocity_ * (dist_to_goal / slowdown_radius_);
        effective_reference_velocity_ =
            std::max(effective_reference_velocity_, reference_velocity_ * 0.05);
    }

    const double step_dist = effective_reference_velocity_ * dt_;

    references_[0] = InterpolateAlongPath(path, cumulative, arc_origin);

    for (int k = 1; k <= horizon_; ++k) {
        const double target_s = arc_origin + step_dist * static_cast<double>(k);
        references_[static_cast<size_t>(k)] =
            InterpolateAlongPath(path, cumulative, target_s);
    }

    return true;
}

}  // namespace tracking
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
