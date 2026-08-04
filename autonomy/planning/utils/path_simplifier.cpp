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

#include "autonomy/planning/utils/path_simplifier.hpp"

#include <cmath>
#include <vector>

namespace autonomy {
namespace planning {
namespace utils {
namespace {

double PerpendicularDistance(
    const automsgs::msgs::geometry_msgs::PoseStamped& point,
    const automsgs::msgs::geometry_msgs::PoseStamped& line_start,
    const automsgs::msgs::geometry_msgs::PoseStamped& line_end) {
    const double x = point.pose().position().x();
    const double y = point.pose().position().y();
    const double x1 = line_start.pose().position().x();
    const double y1 = line_start.pose().position().y();
    const double x2 = line_end.pose().position().x();
    const double y2 = line_end.pose().position().y();

    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double norm = std::hypot(dx, dy);
    if (norm < 1e-9) {
        return std::hypot(x - x1, y - y1);
    }
    return std::abs(dy * x - dx * y + x2 * y1 - y2 * x1) / norm;
}

template <typename PoseRange>
void DouglasPeuckerRec(
    const PoseRange& points,
    size_t start_idx, size_t end_idx, double epsilon,
    std::vector<bool>& marked) {
    if (end_idx <= start_idx + 1) {
        return;
    }

    double max_distance = 0.0;
    size_t index = start_idx;
    for (size_t i = start_idx + 1; i < end_idx; ++i) {
        const double distance =
            PerpendicularDistance(points[i], points[start_idx], points[end_idx]);
        if (distance > max_distance) {
            max_distance = distance;
            index = i;
        }
    }

    if (max_distance > epsilon) {
        DouglasPeuckerRec(points, start_idx, index, epsilon, marked);
        marked[index] = true;
        DouglasPeuckerRec(points, index, end_idx, epsilon, marked);
    }
}

}  // namespace

automsgs::msgs::nav_msgs::Path SimplifyPath(
    const automsgs::msgs::nav_msgs::Path& path, double epsilon) {
    if (epsilon <= 0.0 || path.poses_size() < 3) {
        return path;
    }

    std::vector<bool> marked(path.poses_size(), false);
    marked.front() = true;
    marked.back() = true;
    DouglasPeuckerRec(path.poses(), 0, path.poses_size() - 1, epsilon, marked);

    automsgs::msgs::nav_msgs::Path simplified;
    *simplified.mutable_header() = path.header();
    for (size_t i = 0; i < path.poses_size(); ++i) {
        if (marked[i]) {
            *simplified.mutable_poses()->Add() = path.poses(i);
        }
    }
    return simplified;
}

}  // namespace utils
}  // namespace planning
}  // namespace autonomy
