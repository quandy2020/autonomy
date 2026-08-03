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

#include "autonomy/control/utils/controller_utils.hpp"

#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace control {
namespace utils {

automsgs::msgs::geometry_msgs::Point CircleSegmentIntersection(
    const automsgs::msgs::geometry_msgs::Point& p1,
    const automsgs::msgs::geometry_msgs::Point& p2, double r) {
    double x1 = p1.x();
    double x2 = p2.x();
    double y1 = p1.y();
    double y2 = p2.y();

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr2 = dx * dx + dy * dy;
    double D = x1 * y2 - x2 * y1;

    double d1 = x1 * x1 + y1 * y1;
    double d2 = x2 * x2 + y2 * y2;
    double dd = d2 - d1;

    automsgs::msgs::geometry_msgs::Point p;
    double sqrt_term = std::sqrt(r * r * dr2 - D * D);
    p.set_x((D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2);
    p.set_y((-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2);
    return p;
}

automsgs::msgs::geometry_msgs::Point LinearInterpolation(
    const automsgs::msgs::geometry_msgs::Point& p1,
    const automsgs::msgs::geometry_msgs::Point& p2, const double target_dist) {
    automsgs::msgs::geometry_msgs::Point result;

    double dx = p2.x() - p1.x();
    double dy = p2.y() - p1.y();
    double d_dist = std::hypot(dx, dy);

    double target_ratio = target_dist / d_dist;

    result.set_x(p1.x() + target_ratio * dx);
    result.set_y(p1.y() + target_ratio * dy);
    return result;
}

automsgs::msgs::geometry_msgs::PoseStamped GetLookAheadPoint(
    double& lookahead_dist,
    const automsgs::msgs::planning_msgs::Path& transformed_plan,
    const bool interpolate_after_goal) {
    const auto& poses = transformed_plan.poses();
    auto goal_pose_it = poses.begin();
    double path_dist = 0.0;
    double interpolation_dist = 0.0;

    bool pose_found = false;
    for (size_t i = 1; i < static_cast<size_t>(poses.size()); i++) {
        const auto& prev_pose = poses.Get(static_cast<int>(i - 1)).pose().position();
        const auto& curr_pose = poses.Get(static_cast<int>(i)).pose().position();

        const double d =
            std::hypot(curr_pose.x() - prev_pose.x(), curr_pose.y() - prev_pose.y());
        if (path_dist + d >= lookahead_dist) {
            goal_pose_it = poses.begin() + static_cast<int>(i);
            pose_found = true;
            break;
        }
        path_dist += d;
    }

    interpolation_dist = lookahead_dist - path_dist;
    if (!pose_found) {
        goal_pose_it = poses.end();
    }

    if (goal_pose_it == transformed_plan.poses().end()) {
        if (interpolate_after_goal) {
            auto last_pose_it = std::prev(transformed_plan.poses().end());
            auto prev_last_pose_it = std::prev(last_pose_it);

            double end_path_orientation = atan2(
                last_pose_it->pose().position().y() -
                    prev_last_pose_it->pose().position().y(),
                last_pose_it->pose().position().x() -
                    prev_last_pose_it->pose().position().x());

            auto projected_position = last_pose_it->pose().position();
            projected_position.set_x(projected_position.x() +
                                     cos(end_path_orientation) * lookahead_dist);
            projected_position.set_y(projected_position.y() +
                                     sin(end_path_orientation) * lookahead_dist);

            const auto interpolated_position = LinearInterpolation(
                last_pose_it->pose().position(), projected_position,
                interpolation_dist);

            automsgs::msgs::geometry_msgs::PoseStamped interpolated_pose;
            *interpolated_pose.mutable_header() = last_pose_it->header();
            *interpolated_pose.mutable_pose()->mutable_position() =
                interpolated_position;
            return interpolated_pose;
        }
        lookahead_dist = path_dist;
        goal_pose_it = std::prev(transformed_plan.poses().end());
    } else if (goal_pose_it != transformed_plan.poses().begin()) {
        auto prev_pose_it = std::prev(goal_pose_it);
        auto point = LinearInterpolation(prev_pose_it->pose().position(),
                                         goal_pose_it->pose().position(),
                                         interpolation_dist);

        double yaw = atan2(point.y() - prev_pose_it->pose().position().y(),
                           point.x() - prev_pose_it->pose().position().x());

        automsgs::msgs::geometry_msgs::PoseStamped pose;
        pose.mutable_header()->set_frame_id(prev_pose_it->header().frame_id());
        *pose.mutable_header()->mutable_stamp() = goal_pose_it->header().stamp();
        *pose.mutable_pose()->mutable_position() = point;
        *pose.mutable_pose()->mutable_orientation() =
            map::costmap_2d::utils::OrientationAroundZAxis(yaw);
        return pose;
    }

    return *goal_pose_it;
}

}  // namespace utils
}  // namespace control
}  // namespace autonomy
