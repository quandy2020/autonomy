/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "navigation/planning/tasks/path_follower.hpp"
#include "autonomy/common/math/math.hpp"

#include <algorithm>
#include <cmath>

#include "navigation/common/pose2d.hpp"

namespace navigation
{
namespace planning
{

PathFollower::PathFollower(PathFollowerOptions options) : options_(options)
{
}

bool PathFollower::Compute(const common::Pose2D& pose, const Waypoint& waypoint,
                           common::Twist2D* command) const
{
    if (command == nullptr)
    {
        return false;
    }
    if (Reached(pose, waypoint))
    {
        *command = common::Twist2D{};
        return true;
    }
    const double dx = waypoint.pose.x() - pose.x();
    const double dy = waypoint.pose.y() - pose.y();
    const double cosine = std::cos(pose.theta());
    const double sine = std::sin(pose.theta());
    const double body_x = cosine * dx + sine * dy;
    const double body_y = -sine * dx + cosine * dy;
    const double heading_error = std::atan2(body_y, body_x);
    command->set_theta(autonomy::common::Clamp(options_.heading_gain * heading_error, -options_.max_angular_radps,
                      options_.max_angular_radps));
    const double alignment = std::max(0.0, std::cos(heading_error));
    command->set_x(autonomy::common::Clamp(options_.position_gain * std::hypot(body_x, body_y) * alignment, 0.0,
                      options_.max_linear_mps));
    command->set_y(0.0);
    return true;
}

bool PathFollower::Reached(const common::Pose2D& pose, const Waypoint& waypoint) const
{
    return common::Distance(pose, waypoint.pose) <= waypoint.tolerance;
}

}  // namespace planning
}  // namespace navigation
