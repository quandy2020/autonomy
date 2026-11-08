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

#include "navigation/behavior/tasks/waypoint_sequence_task.hpp"

#include <utility>

namespace navigation
{
namespace behavior
{

void WaypointSequenceTask::SetWaypoints(std::vector<planning::Waypoint> waypoints)
{
    waypoints_ = std::move(waypoints);
    index_ = 0;
    status_ = TaskStatus::kIdle;
}

void WaypointSequenceTask::Start(TaskContext& context)
{
    index_ = 0;
    status_ = waypoints_.empty() ? TaskStatus::kSucceeded : TaskStatus::kRunning;
    if (status_ == TaskStatus::kRunning && !IssueGoal(index_, context))
    {
        status_ = TaskStatus::kFailed;
    }
}

TaskStatus WaypointSequenceTask::Tick(TaskContext& context)
{
    if (status_ != TaskStatus::kRunning)
    {
        return status_;
    }
    if (!context.goal_reached)
    {
        return status_;
    }
    ++index_;
    if (index_ >= waypoints_.size())
    {
        status_ = TaskStatus::kSucceeded;
        return status_;
    }
    if (!IssueGoal(index_, context))
    {
        status_ = TaskStatus::kFailed;
    }
    return status_;
}

void WaypointSequenceTask::Reset()
{
    waypoints_.clear();
    index_ = 0;
    status_ = TaskStatus::kIdle;
}

std::string_view WaypointSequenceTask::Name() const
{
    return "waypoint_sequence";
}

bool WaypointSequenceTask::IssueGoal(std::size_t index, TaskContext& context)
{
    if (context.session == nullptr || index >= waypoints_.size())
    {
        return false;
    }
    return context.session->SetGoal(MakePoseGoal(waypoints_[index], "waypoint"));
}

}  // namespace behavior
}  // namespace navigation
