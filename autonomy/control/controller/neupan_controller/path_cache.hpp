/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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
 * @file path_cache.hpp
 * @brief Flat SoA cache of a nav_msgs Path for hot-path lookahead queries.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <automsgs/msgs/geometry_msgs/pose2d.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

/**
 * @brief Immutable path geometry in structure-of-arrays form.
 *
 * Built once in @c SetPlan; control ticks only read contiguous doubles
 * (no protobuf nested accessors on the hot path).
 */
class PathCache
{
public:
    /**
     * @brief Rebuild SoA buffers from a Path message.
     * @param[in] path Reference path (same frame as the robot pose).
     */
    void BuildFrom(const automsgs::msgs::nav_msgs::Path& path) {
        const int n = path.poses_size();
        x_.resize(static_cast<size_t>(n));
        y_.resize(static_cast<size_t>(n));
        yaw_.resize(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            const auto& pose = path.poses(i).pose();
            x_[static_cast<size_t>(i)] = pose.position().x();
            y_[static_cast<size_t>(i)] = pose.position().y();
            const auto& q = pose.orientation();
            const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
            const double cosy_cosp =
                1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
            yaw_[static_cast<size_t>(i)] = std::atan2(siny_cosp, cosy_cosp);
        }
        last_closest_index_ = 0;
    }

    /** @brief True when no poses are cached. */
    bool Empty() const {
        return x_.empty();
    }

    /** @brief Number of cached poses. */
    int Size() const {
        return static_cast<int>(x_.size());
    }

    /**
     * @brief Euclidean distance from @p pose to the final path pose.
     * @return Distance [m], or +inf when empty.
     */
    double DistanceToGoal(
        const automsgs::msgs::geometry_msgs::Pose2D& pose) const {
        if (Empty()) {
            return std::numeric_limits<double>::infinity();
        }
        const double dx = x_.back() - pose.x();
        const double dy = y_.back() - pose.y();
        return std::sqrt(dx * dx + dy * dy);
    }

    /**
     * @brief Pure-pursuit lookahead on the cached polyline.
     *
     * Uses a warm-started closest-index search (local window then full scan).
     *
     * @param[in] pose Robot pose.
     * @param[in] lookahead_distance Lookahead arc length [m].
     * @param[out] lookahead_pose Selected pose on the path.
     * @param[out] goal_distance Distance to the final pose [m].
     * @return true if the cache is non-empty.
     */
    bool ComputeLookaheadPose(
        const automsgs::msgs::geometry_msgs::Pose2D& pose,
        double lookahead_distance,
        automsgs::msgs::geometry_msgs::Pose2D* lookahead_pose,
        double* goal_distance) const {
        if (lookahead_pose == nullptr || goal_distance == nullptr || Empty()) {
            return false;
        }
        const int n = Size();
        const double px = pose.x();
        const double py = pose.y();

        int closest = FindClosestIndex(px, py);
        last_closest_index_ = closest;

        const double gdx = x_.back() - px;
        const double gdy = y_.back() - py;
        *goal_distance = std::sqrt(gdx * gdx + gdy * gdy);

        int index = closest;
        double accumulated = 0.0;
        while (index + 1 < n && accumulated < lookahead_distance) {
            const double sx = x_[static_cast<size_t>(index + 1)] -
                              x_[static_cast<size_t>(index)];
            const double sy = y_[static_cast<size_t>(index + 1)] -
                              y_[static_cast<size_t>(index)];
            accumulated += std::sqrt(sx * sx + sy * sy);
            ++index;
        }

        lookahead_pose->set_x(x_[static_cast<size_t>(index)]);
        lookahead_pose->set_y(y_[static_cast<size_t>(index)]);
        lookahead_pose->set_theta(yaw_[static_cast<size_t>(index)]);
        return true;
    }

private:
    int FindClosestIndex(double px, double py) const {
        const int n = Size();
        // Warm-start local window around the previous closest index.
        const int window = 32;
        int best = last_closest_index_;
        if (best < 0 || best >= n) {
            best = 0;
        }
        double best_sq = std::numeric_limits<double>::infinity();

        const int lo = std::max(0, best - window);
        const int hi = std::min(n, best + window + 1);
        for (int i = lo; i < hi; ++i) {
            const double dx = x_[static_cast<size_t>(i)] - px;
            const double dy = y_[static_cast<size_t>(i)] - py;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_sq) {
                best_sq = d2;
                best = i;
            }
        }

        // If the local minimum sits on a window edge, fall back to a full scan.
        if (best == lo || best == hi - 1) {
            for (int i = 0; i < n; ++i) {
                const double dx = x_[static_cast<size_t>(i)] - px;
                const double dy = y_[static_cast<size_t>(i)] - py;
                const double d2 = dx * dx + dy * dy;
                if (d2 < best_sq) {
                    best_sq = d2;
                    best = i;
                }
            }
        }
        return best;
    }

    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> yaw_;
    mutable int last_closest_index_{0};
};

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
