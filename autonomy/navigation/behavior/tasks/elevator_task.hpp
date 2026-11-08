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

/**
 * @file elevator_task.hpp
 * @brief Six-phase elevator primitive.
 */

#pragma once

#include <cstdint>

#include "navigation/behavior/tasks/task_primitive.hpp"

namespace navigation
{
namespace behavior
{

class ElevatorTask final : public TaskPrimitive
{
public:
    enum class Phase
    {
        kIdle = 0,
        kCall,
        kWaitDoor,
        kEnter,
        kSelectFloor,
        kRide,
        kExit,
        kDone,
        kFailed,
    };

    struct Durations
    {
        /** @brief Call-button duration. */
        std::int64_t call_ns{3000};
        /** @brief Door-wait duration. */
        std::int64_t wait_door_ns{5000};
        /** @brief Cabin-entry duration. */
        std::int64_t enter_ns{3000};
        /** @brief Floor-selection duration. */
        std::int64_t select_floor_ns{2000};
        /** @brief Ride duration. */
        std::int64_t ride_ns{10000};
        /** @brief Exit duration. */
        std::int64_t exit_ns{3000};
    };

    /** @brief Set phase durations. */
    void SetDurations(Durations durations);

    /** @brief Set task waypoints. */
    void SetGoals(planning::Waypoint call, planning::Waypoint inside,
                  planning::Waypoint exit);

    /** @brief Begin the task. */
    void Start(TaskContext& context) override;

    /** @brief Advance one tick. */
    TaskStatus Tick(TaskContext& context) override;

    /** @brief Reset local state. */
    void Reset() override;

    /** @brief Stable name. */
    [[nodiscard]] std::string_view Name() const override;

    /** @brief Current phase. */
    [[nodiscard]] Phase CurrentPhase() const;

private:
    /** @brief Transition to @p phase and stamp the phase start. */
    void EnterPhase(Phase phase, TaskContext& context);

    /** @brief Per-phase durations. */
    Durations durations_;
    /** @brief Elevator call/entry pose. */
    planning::Waypoint call_;
    /** @brief Pose inside the cabin. */
    planning::Waypoint inside_;
    /** @brief Pose after exiting. */
    planning::Waypoint exit_;
    /** @brief Current state-machine phase. */
    Phase phase_{Phase::kIdle};
    /** @brief Monotonic start time of the current phase. */
    std::int64_t phase_start_ns_{0};
    /** @brief True once all three waypoints were set. */
    bool has_goals_{false};
};

}  // namespace behavior
}  // namespace navigation
