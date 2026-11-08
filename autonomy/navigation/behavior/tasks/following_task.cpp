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

#include "navigation/behavior/tasks/following_task.hpp"

#include "navigation/common/pose2d.hpp"

namespace navigation
{
namespace behavior
{

FollowingTask::FollowingTask(double desired_distance)
    : desired_distance_(desired_distance > 0.0 ? desired_distance : 1.0)
{
}

void FollowingTask::SetTarget(const common::Pose2D& target)
{
    target_ = target;
    target_valid_ = true;
}

void FollowingTask::SetTargetValid(bool valid)
{
    target_valid_ = valid;
}

void FollowingTask::Start(TaskContext& context)
{
    goal_issued_ = false;
    status_ = context.session != nullptr ? TaskStatus::kRunning : TaskStatus::kFailed;
    if (status_ == TaskStatus::kRunning && target_valid_)
    {
        planning::Waypoint waypoint;
        waypoint.pose = target_;
        goal_issued_ = context.session->SetGoal(MakePoseGoal(waypoint, "following_target"));
    }
}

TaskStatus FollowingTask::Tick(TaskContext& context)
{
    if (status_ != TaskStatus::kRunning)
    {
        return status_;
    }
    if (!target_valid_)
    {
        status_ = TaskStatus::kFailed;
        return status_;
    }
    if (common::Distance(context.pose, target_) > desired_distance_ + 0.2 && !goal_issued_)
    {
        planning::Waypoint waypoint;
        waypoint.pose = target_;
        goal_issued_ = context.session != nullptr &&
                       context.session->SetGoal(MakePoseGoal(waypoint, "following_target"));
    }
    if (common::Distance(context.pose, target_) > desired_distance_ + 0.2 && goal_issued_)
    {
        goal_issued_ = false;
    }
    return status_;
}

void FollowingTask::Reset()
{
    target_valid_ = false;
    goal_issued_ = false;
    status_ = TaskStatus::kIdle;
}

std::string_view FollowingTask::Name() const
{
    return "following";
}

}  // namespace behavior
}  // namespace navigation
