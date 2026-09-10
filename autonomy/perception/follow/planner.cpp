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

#include "autonomy/perception/follow/planner.hpp"

#include <cmath>
#include <string>

namespace autonomy {
namespace perception {
namespace follow {
namespace {

void SetError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = "Follow: " + message;
    }
}

}  // namespace

Planner::Planner(proto::FollowOptions options) : options_(std::move(options)) {}

bool Planner::Plan(const automsgs::msgs::geometry_msgs::PoseStamped& robot,
                   const automsgs::msgs::geometry_msgs::PoseStamped& target,
                   const LocalGrid& grid,
                   automsgs::msgs::nav_msgs::Path* path,
                   std::string* error) const {
    if (path == nullptr) {
        SetError(error, "path must not be null.");
        return false;
    }
    path->Clear();
    path->mutable_header()->CopyFrom(robot.header());
    path->mutable_header()->set_frame_id(options_.map_frame());

    const double rx = robot.pose().position().x();
    const double ry = robot.pose().position().y();
    const double tx = target.pose().position().x();
    const double ty = target.pose().position().y();
    const double dx = tx - rx;
    const double dy = ty - ry;
    const double dist = std::hypot(dx, dy);
    if (!(dist > 1e-3)) {
        return true;  // empty path: already at target
    }

    const double goal_dist =
        std::max(0.0, dist - static_cast<double>(options_.follow_distance_m()));
    const double ux = dx / dist;
    const double uy = dy / dist;
    const double yaw = std::atan2(uy, ux);
    const double qz = std::sin(0.5 * yaw);
    const double qw = std::cos(0.5 * yaw);

    const double step = options_.path_step_m();
    const uint32_t max_poses = options_.path_max_poses();
    for (uint32_t i = 0; i < max_poses; ++i) {
        const double s = std::min(goal_dist, step * static_cast<double>(i));
        const double x = rx + ux * s;
        const double y = ry + uy * s;
        if (!grid.IsTraversable(x, y)) {
            break;
        }
        auto* pose = path->add_poses();
        pose->mutable_header()->CopyFrom(path->header());
        pose->mutable_pose()->mutable_position()->set_x(x);
        pose->mutable_pose()->mutable_position()->set_y(y);
        pose->mutable_pose()->mutable_position()->set_z(
            robot.pose().position().z());
        pose->mutable_pose()->mutable_orientation()->set_z(qz);
        pose->mutable_pose()->mutable_orientation()->set_w(qw);
        if (s + 1e-6 >= goal_dist) {
            break;
        }
    }
    return true;
}

}  // namespace follow
}  // namespace perception
}  // namespace autonomy
