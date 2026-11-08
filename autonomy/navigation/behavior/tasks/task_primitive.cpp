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

#include "navigation/behavior/tasks/task_primitive.hpp"

#include <cmath>

namespace navigation
{
namespace behavior
{

GoalSpec MakePoseGoal(const planning::Waypoint& waypoint, const std::string& goal_id)
{
    GoalSpec goal;
    goal.set_type(GoalSpec::TYPE_POSE);
    goal.set_goal_id(goal_id);
    goal.set_position_tolerance(waypoint.tolerance);
    auto* pose = goal.mutable_pose();
    pose->mutable_header()->set_frame_id("map");
    pose->mutable_pose()->mutable_position()->set_x(waypoint.pose.x());
    pose->mutable_pose()->mutable_position()->set_y(waypoint.pose.y());
    pose->mutable_pose()->mutable_orientation()->set_z(std::sin(0.5 * waypoint.pose.theta()));
    pose->mutable_pose()->mutable_orientation()->set_w(std::cos(0.5 * waypoint.pose.theta()));
    return goal;
}

}  // namespace behavior
}  // namespace navigation
