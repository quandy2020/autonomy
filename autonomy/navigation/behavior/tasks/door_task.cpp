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

#include "navigation/behavior/tasks/door_task.hpp"

#include <utility>

namespace navigation
{
namespace behavior
{

void DoorTask::SetDurations(Durations durations)
{
    durations_ = durations;
}

void DoorTask::SetGoals(planning::Waypoint approach, planning::Waypoint pass)
{
    approach_ = std::move(approach);
    pass_ = std::move(pass);
    has_goals_ = true;
}

void DoorTask::Start(TaskContext& context)
{
    if (!has_goals_ || context.session == nullptr)
    {
        phase_ = Phase::kFailed;
        return;
    }
    phase_start_ns_ = context.now_ns;
    EnterPhase(Phase::kDetect, context);
}

TaskStatus DoorTask::Tick(TaskContext& context)
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
        case Phase::kDetect:
            duration = durations_.detect_ns;
            break;
        case Phase::kApproach:
            duration = durations_.approach_ns;
            break;
        case Phase::kOpen:
            duration = durations_.open_ns;
            break;
        case Phase::kPass:
            duration = durations_.pass_ns;
            break;
        case Phase::kRelease:
            duration = durations_.release_ns;
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
        case Phase::kDetect:
            EnterPhase(Phase::kApproach, context);
            break;
        case Phase::kApproach:
            EnterPhase(Phase::kOpen, context);
            break;
        case Phase::kOpen:
            EnterPhase(Phase::kPass, context);
            break;
        case Phase::kPass:
            EnterPhase(Phase::kRelease, context);
            break;
        case Phase::kRelease:
            phase_ = Phase::kDone;
            return TaskStatus::kSucceeded;
        default:
            break;
    }
    return TaskStatus::kRunning;
}

void DoorTask::Reset()
{
    phase_ = Phase::kIdle;
    phase_start_ns_ = 0;
}

std::string_view DoorTask::Name() const
{
    return "door";
}

DoorTask::Phase DoorTask::CurrentPhase() const
{
    return phase_;
}

void DoorTask::EnterPhase(Phase phase, TaskContext& context)
{
    phase_ = phase;
    if (context.session == nullptr)
    {
        phase_ = Phase::kFailed;
        return;
    }
    switch (phase)
    {
        case Phase::kApproach:
            (void)context.session->SetGoal(MakePoseGoal(approach_, "door_approach"));
            break;
        case Phase::kPass:
            (void)context.session->SetGoal(MakePoseGoal(pass_, "door_pass"));
            break;
        case Phase::kDetect:
        case Phase::kOpen:
        case Phase::kRelease:
            context.session->ClearGoal();
            break;
        default:
            break;
    }
}

}  // namespace behavior
}  // namespace navigation
