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

#include "navigation/planning/plugins/policy/vntpa/query_builder.hpp"

#include "navigation/common/habitat_pose.hpp"

namespace navigation
{
namespace planning
{
namespace vntpa
{

bool QueryBuilder::Build(const common::Pose2D& current_pose, const GoalSpec& goal,
                         std::vector<float>* query_pose, std::vector<float>* goal_xy,
                         bool* has_goal) const
{
    if (query_pose == nullptr || goal_xy == nullptr || has_goal == nullptr)
    {
        return false;
    }
    const common::Pose2D habitat = common::RosToHabitat(current_pose);
    query_pose->assign({static_cast<float>(habitat.x()),
                        static_cast<float>(habitat.y()),
                        static_cast<float>(habitat.theta())});
    *has_goal = goal.has_pose();
    if (!*has_goal)
    {
        goal_xy->assign({0.0F, 0.0F});
        return true;
    }
    const auto& position = goal.pose().pose().position();
    common::Pose2D goal_pose;
    goal_pose.set_x(position.x());
    goal_pose.set_y(position.y());
    const common::Pose2D habitat_goal = common::RosToHabitat(goal_pose);
    goal_xy->assign({static_cast<float>(habitat_goal.x()),
                     static_cast<float>(habitat_goal.y())});
    return true;
}

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
