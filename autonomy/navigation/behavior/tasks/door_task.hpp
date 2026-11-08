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
 * @file door_task.hpp
 * @brief Five-phase semantic door primitive.
 */

#pragma once

#include <cstdint>

#include "navigation/behavior/tasks/task_primitive.hpp"

namespace navigation
{
namespace behavior
{

class DoorTask final : public TaskPrimitive
{
public:
    enum class Phase
    {
        kIdle = 0,
        kDetect,
        kApproach,
        kOpen,
        kPass,
        kRelease,
        kDone,
        kFailed,
    };

    struct Durations
    {
        /** @brief Door detection duration. */
        std::int64_t detect_ns{2000};
        /** @brief Approach duration. */
        std::int64_t approach_ns{4000};
        /** @brief Door-open wait duration. */
        std::int64_t open_ns{3000};
        /** @brief Pass-through duration. */
        std::int64_t pass_ns{4000};
        /** @brief Release duration. */
        std::int64_t release_ns{1000};
    };

    /** @brief Set phase durations. */
    void SetDurations(Durations durations);

    /** @brief Set task waypoints. */
    void SetGoals(planning::Waypoint approach, planning::Waypoint pass);

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
    /** @brief Pose in front of the door. */
    planning::Waypoint approach_;
    /** @brief Pose beyond the door. */
    planning::Waypoint pass_;
    /** @brief Current state-machine phase. */
    Phase phase_{Phase::kIdle};
    /** @brief Monotonic start time of the current phase. */
    std::int64_t phase_start_ns_{0};
    /** @brief True once both waypoints were set. */
    bool has_goals_{false};
};

}  // namespace behavior
}  // namespace navigation
