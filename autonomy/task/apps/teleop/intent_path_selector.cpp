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

#include "autonomy/task/apps/teleop/intent_path_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "autonomy/map/costmap_2d/cost_values.hpp"

namespace autonomy::task::teleop {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDirectionLimitDeg = 120.0;
constexpr double kStoppedSpeedEpsilon = 1e-6;

automsgs::msgs::geometry_msgs::PoseStamped MakePose(double x, double y, double yaw) {
    automsgs::msgs::geometry_msgs::PoseStamped pose{};
    pose.mutable_pose()->mutable_position()->set_x(x);
    pose.mutable_pose()->mutable_position()->set_y(y);
    pose.mutable_pose()->mutable_orientation()->set_z(std::sin(yaw * 0.5));
    pose.mutable_pose()->mutable_orientation()->set_w(std::cos(yaw * 0.5));
    return pose;
}

}  // namespace

void IntentPathSelector::GenerateDefaultLibrary(int num_dirs, int num_lengths,
                                                double max_range, double ds) {
    candidates_.clear();
    if (num_dirs <= 0 || num_lengths <= 0 || max_range <= 0.0 || ds <= 0.0) {
        return;
    }

    candidates_.reserve(static_cast<std::size_t>(num_dirs * num_lengths));
    for (int dir_index = 0; dir_index < num_dirs; ++dir_index) {
        const double end_dir_deg =
            num_dirs == 1
                ? 0.0
                : -90.0 + 180.0 * dir_index / static_cast<double>(num_dirs - 1);
        const double end_dir_rad = end_dir_deg * kPi / 180.0;

        for (int length_index = 1; length_index <= num_lengths;
             ++length_index) {
            const double length =
                max_range * length_index / static_cast<double>(num_lengths);
            const double curvature = end_dir_rad / length;

            PathCandidate candidate;
            candidate.end_dir_deg = end_dir_deg;
            const int num_steps = static_cast<int>(std::ceil(length / ds));
            candidate.path.mutable_poses()->Reserve(
                static_cast<int>(num_steps) + 1);
            *candidate.path.add_poses() = MakePose(0.0, 0.0, 0.0);

            for (int step = 1; step <= num_steps; ++step) {
                const double arc_length = std::min(step * ds, length);
                const double yaw = curvature * arc_length;
                if (std::abs(curvature) < 1e-12) {
                    *candidate.path.add_poses() =
                        MakePose(arc_length, 0.0, yaw);
                } else {
                    *candidate.path.add_poses() =
                        MakePose(std::sin(yaw) / curvature,
                                 (1.0 - std::cos(yaw)) / curvature, yaw);
                }
            }
            candidates_.push_back(std::move(candidate));
        }
    }
}

std::optional<automsgs::msgs::nav_msgs::Path> IntentPathSelector::Select(
    const map::costmap_2d::Costmap2D& costmap, double joy_dir_deg,
    double joy_speed) const {
    if (std::abs(joy_speed) <= kStoppedSpeedEpsilon || candidates_.empty()) {
        return std::nullopt;
    }

    const PathCandidate* best_candidate = nullptr;
    double best_score = -std::numeric_limits<double>::infinity();
    for (const auto& candidate : candidates_) {
        const int lethal_hits = CountLethalHits(costmap, candidate.path);
        if (lethal_hits >= point_per_path_thre_) {
            continue;
        }

        const double dir_diff =
            std::min(std::abs(WrapDeg(candidate.end_dir_deg - joy_dir_deg)),
                     kDirectionLimitDeg);
        const double direction_score =
            1.0 - std::sqrt(dir_diff / kDirectionLimitDeg);
        const double clearance_score =
            1.0 - static_cast<double>(lethal_hits) / point_per_path_thre_;

        // CMU local_planner first builds clearPathList, then uses dirDiff.
        // Preserve that safety priority: clearance dominates, while direction
        // breaks ties and keeps the selected path aligned with joystick intent.
        const double score = clearance_score + dir_weight_ * direction_score;
        if (score > best_score) {
            best_score = score;
            best_candidate = &candidate;
        }
    }

    if (best_candidate == nullptr) {
        return std::nullopt;
    }
    return best_candidate->path;
}

int IntentPathSelector::CountLethalHits(
    const map::costmap_2d::Costmap2D& map,
    const automsgs::msgs::nav_msgs::Path& path) {
    int hits = 0;
    for (const auto& pose : path.poses()) {
        unsigned int map_x = 0;
        unsigned int map_y = 0;
        if (map.worldToMap(pose.pose().position().x(), pose.pose().position().y(), map_x,
                           map_y) &&
            map.getCost(map_x, map_y) >= map::costmap_2d::LETHAL_OBSTACLE) {
            ++hits;
        }
    }
    return hits;
}

double IntentPathSelector::WrapDeg(double deg) {
    deg = std::fmod(deg + 180.0, 360.0);
    if (deg < 0.0) {
        deg += 360.0;
    }
    return deg - 180.0;
}

}  // namespace autonomy::task::teleop
