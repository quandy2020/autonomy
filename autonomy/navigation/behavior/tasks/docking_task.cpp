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

#include "navigation/behavior/tasks/docking_task.hpp"

#include <utility>

namespace navigation
{
namespace behavior
{

void DockingTask::SetDurations(Durations durations)
{
    durations_ = durations;
}

void DockingTask::SetDock(planning::Waypoint dock)
{
    dock_ = std::move(dock);
    has_dock_ = true;
}

void DockingTask::Start(TaskContext& context)
{
    if (!has_dock_ || context.session == nullptr)
    {
        phase_ = Phase::kFailed;
        return;
    }
    phase_start_ns_ = context.now_ns;
    EnterPhase(Phase::kSearch, context);
}

TaskStatus DockingTask::Tick(TaskContext& context)
{
    if (phase_ == Phase::kDone)
    {
        return TaskStatus::kSucceeded;
    }
    if (phase_ == Phase::kFailed)
    {
        return TaskStatus::kFailed;
    }
    if (phase_ == Phase::kIdle)
    {
        return TaskStatus::kIdle;
    }
    const std::int64_t elapsed = context.now_ns - phase_start_ns_;
    if (elapsed < 0)
    {
        phase_ = Phase::kFailed;
        return TaskStatus::kFailed;
    }
    std::int64_t duration = 0;
    switch (phase_)
    {
        case Phase::kSearch:
            duration = durations_.search_ns;
            break;
        case Phase::kApproach:
            duration = durations_.approach_ns;
            break;
        case Phase::kAlign:
            duration = durations_.align_ns;
            break;
        case Phase::kFinalApproach:
            duration = durations_.final_ns;
            break;
        case Phase::kVerify:
            duration = durations_.verify_ns;
            break;
        default:
            break;
    }
    if (elapsed < duration)
    {
        return TaskStatus::kRunning;
    }
    phase_start_ns_ = context.now_ns;
    switch (phase_)
    {
        case Phase::kSearch:
            EnterPhase(Phase::kApproach, context);
            break;
        case Phase::kApproach:
            EnterPhase(Phase::kAlign, context);
            break;
        case Phase::kAlign:
            EnterPhase(Phase::kFinalApproach, context);
            break;
        case Phase::kFinalApproach:
            EnterPhase(Phase::kVerify, context);
            break;
        case Phase::kVerify:
            phase_ = Phase::kDone;
            return TaskStatus::kSucceeded;
        default:
            break;
    }
    return TaskStatus::kRunning;
}

void DockingTask::Reset()
{
    phase_ = Phase::kIdle;
    phase_start_ns_ = 0;
}

std::string_view DockingTask::Name() const
{
    return "docking";
}

DockingTask::Phase DockingTask::CurrentPhase() const
{
    return phase_;
}

void DockingTask::EnterPhase(Phase phase, TaskContext& context)
{
    phase_ = phase;
    if (context.session == nullptr)
    {
        phase_ = Phase::kFailed;
        return;
    }
    if (phase == Phase::kApproach || phase == Phase::kFinalApproach)
    {
        (void)context.session->SetGoal(MakePoseGoal(dock_, "dock"));
    }
}

}  // namespace behavior
}  // namespace navigation
