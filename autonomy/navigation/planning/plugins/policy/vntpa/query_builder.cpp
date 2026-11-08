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

#include <cmath>

#include "navigation/common/pose2d.hpp"

namespace navigation
{
namespace planning
{
namespace vntpa
{

QueryBuilder::QueryBuilder(std::uint32_t output_dim) : pose_attention_(output_dim)
{
}

bool QueryBuilder::Build(const common::Pose2D& current_pose, const GoalSpec& goal,
                         std::vector<float>* query, bool* has_goal)
{
    if (query == nullptr || has_goal == nullptr)
    {
        return false;
    }
    *has_goal = goal.has_pose();
    if (!*has_goal)
    {
        query->assign(pose_attention_.OutputDim(), 0.0F);
        return true;
    }
    const auto& target = goal.pose();
    common::Pose2D goal_pose;
    goal_pose.set_x(target.pose().position().x());
    goal_pose.set_y(target.pose().position().y());
    const auto& orientation = target.pose().orientation();
    goal_pose.set_theta(std::atan2(2.0 * (orientation.w() * orientation.z() +
                          orientation.x() * orientation.y()),
                   1.0 - 2.0 * (orientation.y() * orientation.y() +
                                orientation.z() * orientation.z())));
    const common::Pose2D relative = common::Compose(common::Inverse(current_pose), goal_pose);
    return pose_attention_.Encode(relative, query);
}

std::uint32_t QueryBuilder::OutputDim() const
{
    return pose_attention_.OutputDim();
}

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
