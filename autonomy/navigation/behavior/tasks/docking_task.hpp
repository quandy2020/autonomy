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
 * @file docking_task.hpp
 * @brief Five-phase docking primitive.
 */

#pragma once

#include <cstdint>

#include "navigation/behavior/tasks/task_primitive.hpp"

namespace navigation
{
namespace behavior
{

class DockingTask final : public TaskPrimitive
{
public:
    enum class Phase
    {
        kIdle = 0,
        kSearch,
        kApproach,
        kAlign,
        kFinalApproach,
        kVerify,
        kDone,
        kFailed,
    };

    struct Durations
    {
        /** @brief Search phase duration. */
        std::int64_t search_ns{2000};
        /** @brief Approach phase duration. */
        std::int64_t approach_ns{4000};
        /** @brief Align phase duration. */
        std::int64_t align_ns{2000};
        /** @brief Final docking duration. */
        std::int64_t final_ns{2000};
        /** @brief Verification duration. */
        std::int64_t verify_ns{1000};
    };

    /** @brief Set phase durations. */
    void SetDurations(Durations durations);

    /** @brief Set docking target. */
    void SetDock(planning::Waypoint dock);

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
    /** @brief Final docking pose. */
    planning::Waypoint dock_;
    /** @brief Current state-machine phase. */
    Phase phase_{Phase::kIdle};
    /** @brief Monotonic start time of the current phase. */
    std::int64_t phase_start_ns_{0};
    /** @brief True once a dock target was set. */
    bool has_dock_{false};
};

}  // namespace behavior
}  // namespace navigation
