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
 * @file task_primitive.hpp
 * @brief State-machine boundary for behavior task primitives.
 */

#pragma once

#include <cstdint>
#include <string_view>

#include "navigation/common/pose2d.hpp"
#include "navigation/planning/base/waypoint.hpp"
#include "navigation/planning/interface/navigation_session.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace behavior
{

/** @brief Task primitive lifecycle state. */
enum class TaskStatus
{
    kIdle = 0,   ///< Not started.
    kRunning,    ///< In progress.
    kSucceeded,  ///< Completed successfully.
    kFailed,     ///< Terminated with failure.
};

/** @brief Per-tick task inputs. */
struct TaskContext
{
    /** @brief Navigation session; borrowed. */
    planning::NavigationSession* session{nullptr};
    /** @brief Current map-frame pose. */
    common::Pose2D pose;
    /** @brief Monotonic time in nanoseconds. */
    std::int64_t now_ns{0};
    /** @brief Tick period in seconds. */
    double dt_s{0.0};
    /** @brief Set when the session reached its goal. */
    bool goal_reached{false};
};

class TaskPrimitive
{
public:
    virtual ~TaskPrimitive() = default;

    /** @brief Begin the task and issue the first goal/action. */
    virtual void Start(TaskContext& context) = 0;

    /** @brief Advance one tick; returns the new lifecycle state. */
    virtual TaskStatus Tick(TaskContext& context) = 0;

    /** @brief Drop all task-local state. */
    virtual void Reset() = 0;

    /** @brief Stable task name for tests/diagnostics. */
    [[nodiscard]] virtual std::string_view Name() const = 0;
};

/** @brief Convert a waypoint to a TYPE_POSE goal. */
GoalSpec MakePoseGoal(const planning::Waypoint& waypoint, const std::string& goal_id = "");

}  // namespace behavior
}  // namespace navigation
