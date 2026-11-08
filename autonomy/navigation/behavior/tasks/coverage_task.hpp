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
 * @file coverage_task.hpp
 * @brief Region coverage delegated to WaypointSequenceTask.
 */

#pragma once

#include "navigation/behavior/tasks/task_primitive.hpp"
#include "navigation/behavior/tasks/waypoint_sequence_task.hpp"
#include "navigation/planning/tasks/coverage_planner.hpp"
#include "navigation/proto/navigation.pb.h"

namespace navigation
{
namespace behavior
{

class CoverageTask final : public TaskPrimitive
{
public:
    /** @brief Set coverage region. */
    bool SetRegion(const Region& region);

    /** @brief Begin the task. */
    void Start(TaskContext& context) override;

    /** @brief Advance one tick. */
    TaskStatus Tick(TaskContext& context) override;

    /** @brief Reset local state. */
    void Reset() override;

    /** @brief Stable name. */
    [[nodiscard]] std::string_view Name() const override;

private:
    /** @brief Boustrophedon planner. */
    planning::CoveragePlanner planner_;
    /** @brief Executes the generated waypoint list. */
    WaypointSequenceTask sequence_;
    /** @brief True once a region has been planned. */
    bool has_region_{false};
};

}  // namespace behavior
}  // namespace navigation
