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

#include "navigation/behavior/tasks/elevator_task.hpp"

#include <utility>

namespace navigation
{
namespace behavior
{

void ElevatorTask::SetDurations(Durations durations)
{
    durations_ = durations;
}

void ElevatorTask::SetGoals(planning::Waypoint call, planning::Waypoint inside,
                            planning::Waypoint exit)
{
    call_ = std::move(call);
    inside_ = std::move(inside);
    exit_ = std::move(exit);
    has_goals_ = true;
}

void ElevatorTask::Start(TaskContext& context)
{
    if (!has_goals_ || context.session == nullptr)
    {
        phase_ = Phase::kFailed;
        return;
    }
    phase_start_ns_ = context.now_ns;
    EnterPhase(Phase::kCall, context);
}

TaskStatus ElevatorTask::Tick(TaskContext& context)
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
    if (context.now_ns - phase_start_ns_ < 0)
    {
        phase_ = Phase::kFailed;
        return TaskStatus::kFailed;
    }
    const std::int64_t elapsed = context.now_ns - phase_start_ns_;
    std::int64_t duration = 0;
    switch (phase_)
    {
        case Phase::kCall:
            duration = durations_.call_ns;
            break;
        case Phase::kWaitDoor:
            duration = durations_.wait_door_ns;
            break;
        case Phase::kEnter:
            duration = durations_.enter_ns;
            break;
        case Phase::kSelectFloor:
            duration = durations_.select_floor_ns;
            break;
        case Phase::kRide:
            duration = durations_.ride_ns;
            break;
        case Phase::kExit:
            duration = durations_.exit_ns;
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
        case Phase::kCall:
            EnterPhase(Phase::kWaitDoor, context);
            break;
        case Phase::kWaitDoor:
            EnterPhase(Phase::kEnter, context);
            break;
        case Phase::kEnter:
            EnterPhase(Phase::kSelectFloor, context);
            break;
        case Phase::kSelectFloor:
            EnterPhase(Phase::kRide, context);
            break;
        case Phase::kRide:
            EnterPhase(Phase::kExit, context);
            break;
        case Phase::kExit:
            phase_ = Phase::kDone;
            return TaskStatus::kSucceeded;
        default:
            break;
    }
    return TaskStatus::kRunning;
}

void ElevatorTask::Reset()
{
    phase_ = Phase::kIdle;
    phase_start_ns_ = 0;
}

std::string_view ElevatorTask::Name() const
{
    return "elevator";
}

ElevatorTask::Phase ElevatorTask::CurrentPhase() const
{
    return phase_;
}

void ElevatorTask::EnterPhase(Phase phase, TaskContext& context)
{
    phase_ = phase;
    if (context.session == nullptr)
    {
        phase_ = Phase::kFailed;
        return;
    }
    switch (phase)
    {
        case Phase::kCall:
            (void)context.session->SetGoal(MakePoseGoal(call_, "elevator_call"));
            break;
        case Phase::kEnter:
            (void)context.session->SetGoal(MakePoseGoal(inside_, "elevator_enter"));
            break;
        case Phase::kExit:
            (void)context.session->SetGoal(MakePoseGoal(exit_, "elevator_exit"));
            break;
        case Phase::kWaitDoor:
        case Phase::kSelectFloor:
        case Phase::kRide:
            context.session->ClearGoal();
            break;
        default:
            break;
    }
}

}  // namespace behavior
}  // namespace navigation
